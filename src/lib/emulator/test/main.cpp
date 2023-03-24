#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>

#include <lib/emulator/emulator.h>

#include <thirdparty/include/nlohmann/json.hpp>

using json = nlohmann::json;

namespace {

using TRamSnapshot = std::vector<std::pair<TAddressType, TByte>>;

class TTestDevice : public NMemory::IDevice {
public:
    TTestDevice(int pc, const json& prefetch, const json& ram) {
        // remember the PC
        PC_ = pc;

        // fill RAM
        for (const auto& pair : ram) {
            const auto addr = pair[0].get<int>();
            const auto value = pair[1].get<TByte>();
            Values_[addr] = value;
        }

        // fill prefetch
        assert(prefetch.size() == 2);
        Prefetch_.push_back(prefetch[0].get<int>() >> 8);
        Prefetch_.push_back(prefetch[0].get<int>() % 256);
        Prefetch_.push_back(prefetch[1].get<int>() >> 8);
        Prefetch_.push_back(prefetch[1].get<int>() % 256);
    }

    tl::expected<TDataHolder, TError> Read(TAddressType addr, TAddressType size) override {
        if (addr >= PC_ && addr + size - PC_ <= 4) {
            TDataHolder res;
            for (int i = 0; i < size; ++i) {
                res.push_back(Prefetch_[addr + i - PC_]);
            }
            return res;
        }

        if (addr % size != 0) {
            return tl::unexpected<TError>(TError::UnalignedMemoryAccess, "memory access at address %#08x of size %d", addr, size);
        }

        TDataHolder res;
        for (int i = addr; i < addr + size; ++i) {
            if (const auto it = Values_.find(i & 0xFFFFFF); it != Values_.end()) {
                res.push_back(it->second);
            } else {
                res.push_back(0);
            }
        }
        return res;
    }

    std::optional<TError> Write(TAddressType addr, TDataView data) override {
        for (const auto value : data) {
            const auto realAddr = addr & 0xFFFFFF;
            if (value != 0 || Values_.contains(realAddr)) {
                Values_[realAddr] = value;
            }
            addr++;
        }
        return std::nullopt;
    }

    TRamSnapshot MakeRamSnapshot() const {
        TRamSnapshot res;
        for (const auto& p : Values_) {
            res.emplace_back(p.first, p.second);
        }
        return res;
    }

private:
    int PC_;
    std::vector<TByte> Prefetch_;
    std::map<TAddressType, TByte> Values_;
};

std::string DumpRamSnapshot(const TRamSnapshot& ram) {
    std::stringstream ss;
    for (const auto& pair : ram) {
        ss << "[" << pair.first << "] = " << static_cast<int>(pair.second) << "\n";
    }
    return ss.str();
}

TRamSnapshot MakeRamSnapshot(const json& ram) {
    std::map<TAddressType, TByte> values;
    for (const auto& pair : ram) {
        const auto addr = pair[0].get<int>();
        const auto value = pair[1].get<TByte>();
        values[addr] = value;
    }

    TRamSnapshot res;
    for (const auto& p : values) {
        res.emplace_back(p.first, p.second);
    }
    return res;
}

json LoadTestFile(std::string_view path) {
    std::ifstream f(path.data());
    json data = json::parse(f);
    std::cerr << "\"" << path << "\" parsed" << std::endl;
    return data;
}

std::optional<std::string> DumpDiff(const NRegisters::TRegisters& lhs, const NRegisters::TRegisters& rhs) {
    std::vector<std::string> diffs;

    for (int i = 0; i < 8; ++i) {
        if (lhs.D[i] != rhs.D[i]) diffs.emplace_back("D" + std::to_string(i));
    }
    for (int i = 0; i < 7; ++i) {
        if (lhs.A[i] != rhs.A[i]) diffs.emplace_back("A" + std::to_string(i));
    }
    if (lhs.USP != rhs.USP) diffs.emplace_back("USP");
    if (lhs.SSP != rhs.SSP) diffs.emplace_back("SSP");
    if (lhs.PC != rhs.PC) diffs.emplace_back("PC");
    if (lhs.SR != rhs.SR) diffs.emplace_back("SR");

    if (diffs.empty()) {
        return std::nullopt;
    } else {
        std::string info = "";
        for (const auto& d : diffs) {
            info += d + " ";
        }
        return info;
    }
}

NRegisters::TRegisters ParseRegisters(const json& j) {
    NRegisters::TRegisters r;
    for (int i = 0; i < 8; ++i) {
        r.D[i] = j["d" + std::to_string(i)].get<int>();
    }
    for (int i = 0; i < 7; ++i) {
        r.A[i] = j["a" + std::to_string(i)].get<int>();
    }
    r.USP = j["usp"].get<int>();
    r.SSP = j["ssp"].get<int>();
    r.SR = j["sr"].get<int>();
    r.PC = j["pc"].get<int>();
    return r;
}

bool WorkOnTest(const json& test) {
    const auto& initialJson = test["initial"];
    const auto& finalJson = test["final"];

    auto initial = ParseRegisters(initialJson);
    auto expectedFinal = ParseRegisters(finalJson);

    auto actualFinal = initial;
    TTestDevice device{initial.PC, initialJson["prefetch"], initialJson["ram"]};
    NEmulator::TContext ctx{.Registers = actualFinal, .Memory = device};
    const auto err = NEmulator::Emulate(ctx);

    if (err) {
        std::cerr << "Got error: " << err->GetWhat() << std::endl;

        // this program counter means there really was an illegal instruction
        return (expectedFinal.PC == 0x1400);
    }

    // TODO: compare RAM
    const auto regsDiff = DumpDiff(expectedFinal, actualFinal);
    const auto actualRam = MakeRamSnapshot(device.MakeRamSnapshot());
    const auto expectedRam = MakeRamSnapshot(finalJson["ram"]);

    if (regsDiff || expectedRam != actualRam) {
        std::cerr << "Test name: \"" << test["name"].get<std::string>() << "\"" << std::endl << std::endl;

        if (regsDiff || true) {
            std::cerr << "Initial registers:" << std::endl;
            std::cerr << Dump(initial) << std::endl;

            std::cerr << "Actual final registers:" << std::endl;
            std::cerr << Dump(actualFinal) << std::endl;

            std::cerr << "Expected final registers:" << std::endl;
            std::cerr << Dump(expectedFinal) << std::endl;

            std::cerr << "Differing registers: " << *regsDiff << std::endl << std::endl;
        }

        if (expectedRam != actualRam || true) {
            std::cerr << "Initial RAM:" << std::endl;
            std::cerr << DumpRamSnapshot(MakeRamSnapshot(initialJson["ram"])) << std::endl;

            std::cerr << "Actual RAM:" << std::endl;
            std::cerr << DumpRamSnapshot(actualRam) << std::endl;

            std::cerr << "Expected RAM:" << std::endl;
            std::cerr << DumpRamSnapshot(expectedRam) << std::endl;

            std::cerr << "RAM differs" << std::endl;
        }

        return false;
    }

    return true;
}

bool WorkOnFile(const json& file) {
    std::size_t size = file.size();
    std::cerr << "work on file with " << size << " tests" << std::endl;

    int passed = 0;
    int failed = 0;
    int ignored = size;

    for (std::size_t i = 0; i < size; ++i) {
        const bool ok = WorkOnTest(file[i]);
        std::cerr << (i + 1) << "/" << size << " test is " << (ok ? "OK" : "FAIL") << std::endl;

        --ignored;
        if (ok) {
            ++passed;
        } else {
            ++failed;
            //break;
        }
    }
    std::cerr << "TOTAL TESTS: " << size << std::endl;
    std::cerr << "PASSED TESTS: " << passed << std::endl;
    std::cerr << "FAILED TESTS: " << failed << std::endl;
    std::cerr << "IGNORED TESTS: " << ignored << std::endl;
    return passed == size;
}

} // namespace

int main() {
    namespace fs = std::filesystem;

    std::set<std::string> paths;
    for (const auto& entry : fs::directory_iterator("/home/mango/ProcessorTests/680x0/68000/v1")) {
        auto path = entry.path().string();
        if (!path.ends_with(".json")) {
            continue;
        }
        paths.emplace(std::move(path));
    }

    int skipFiles = 3;
    for (const auto& path : paths) {
        if (skipFiles) {
            --skipFiles;
            continue;
        }
        auto file = LoadTestFile(path);
        if (!WorkOnFile(file)) {
            break;
        }
        break;
    }
}
