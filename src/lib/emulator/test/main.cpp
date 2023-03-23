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
        if (prefetch.size() != 2) {
            throw std::runtime_error("why not 2 items?");
        }
        Prefetch_.push_back(static_cast<TByte>(prefetch[0].get<int>() >> 8));
        Prefetch_.push_back(static_cast<TByte>(prefetch[0].get<int>() % 256));
        Prefetch_.push_back(static_cast<TByte>(prefetch[1].get<int>() >> 8));
        Prefetch_.push_back(static_cast<TByte>(prefetch[1].get<int>() % 256));
    }

    TDataHolder Read(TAddressType addr, TAddressType size) override {
        if (addr == PC_) {
            if (size != 2) {
                throw std::runtime_error("why not 2 bytes?");
            }
            TDataHolder res;
            res.push_back(Prefetch_[0]);
            res.push_back(Prefetch_[1]);
            return res;
        }

        TDataHolder res;
        for (int i = addr; i < addr + size; ++i) {
            if (const auto it = Values_.find(addr & 0xFFFFFF); it != Values_.end()) {
                res.push_back(it->second);
            } else {
                res.push_back(static_cast<TByte>(0));
            }
        }
        return res;
    }

    void Write(TAddressType addr, TDataView data) override {
        for (const auto value : data) {
            if (value != static_cast<TByte>(0)) {
                Values_[addr & 0xFFFFFF] = value;
            }
            addr++;
        }
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
    NEmulator::Emulate(ctx);

    // TODO: compare RAM
    const auto regsDiff = DumpDiff(expectedFinal, actualFinal);
    const auto actualRam = MakeRamSnapshot(device.MakeRamSnapshot());
    const auto expectedRam = MakeRamSnapshot(finalJson["ram"]);

    if (regsDiff || expectedRam != actualRam) {
        std::cerr << "Test name: \"" << test["name"].get<std::string>() << "\"" << std::endl << std::endl;

        if (regsDiff) {
            std::cerr << "Initial registers:" << std::endl;
            std::cerr << Dump(initial) << std::endl;

            std::cerr << "Actual final registers:" << std::endl;
            std::cerr << Dump(actualFinal) << std::endl;

            std::cerr << "Expected final registers:" << std::endl;
            std::cerr << Dump(expectedFinal) << std::endl;

            std::cerr << "Differing registers: " << *regsDiff << std::endl << std::endl;
        }

        if (expectedRam != actualRam) {
            std::cerr << "Initial RAM:" << std::endl;
            std::cerr << DumpRamSnapshot(MakeRamSnapshot(initialJson["ram"])) << std::endl;

            std::cerr << "Actual RAM:" << std::endl;
            std::cerr << DumpRamSnapshot(actualRam) << std::endl;

            std::cerr << "Expected RAM:" << std::endl;
            std::cerr << DumpRamSnapshot(expectedRam) << std::endl;

            std::cerr << "RAM differs" << std::endl;
        }

        //std::cerr << "Test source: \"" << test.dump(4) << std::endl;

        return false;
    }

    return true;
}

bool WorkOnFile(const json& file) {
    std::size_t size = file.size();
    std::cerr << "work on file with " << size << " tests" << std::endl;

    for (std::size_t i = 0; i < size; ++i) {
        const bool ok = WorkOnTest(file[i]);
        std::cerr << (i + 1) << "/" << size << " test is " << (ok ? "OK" : "FAIL") << std::endl;
        if (!ok) {
            return false;
        }
    }
    return true;
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

    for (const auto& path : paths) {
        auto file = LoadTestFile(path);
        if (!WorkOnFile(file)) {
            break;
        }
    }
}
