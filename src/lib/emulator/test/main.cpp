#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <thread>

#include <lib/emulator/emulator.h>

#include <thirdparty/include/nlohmann/json.hpp>

using json = nlohmann::json;

thread_local std::ofstream ferr;

namespace {

using TRamSnapshot = std::map<TAddressType, TByte>;

class TTestDevice : public NMemory::IDevice {
public:
    TTestDevice(TLong pc, const json& prefetch, const json& ram) {
        // fill RAM
        for (const auto& pair : ram) {
            const auto addr = pair[0].get<int>();
            const auto value = pair[1].get<TByte>();
            Values_[addr] = value;
        }

        // fill prefetch
        assert(prefetch.size() == 2);
        Values_[pc    ] = prefetch[0].get<int>() >> 8;
        Values_[pc + 1] = prefetch[0].get<int>() % 256;
        Values_[pc + 2] = prefetch[1].get<int>() >> 8;
        Values_[pc + 3] = prefetch[1].get<int>() % 256;
    }

    tl::expected<TDataHolder, TError> Read(TAddressType addr, TAddressType size) override {
        ferr << "Read memory " << (addr & 0xFFFFFF) << " with size " << size << std::endl;

        if (size > 1 && addr % 2 != 0) {
            return tl::unexpected<TError>(TError::UnalignedMemoryRead, "memory read at address %#08x of size %d", addr, size);
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
        if (data.size() > 1 && addr % 2 != 0) {
            return TError{TError::UnalignedMemoryWrite, "memory write at address %#08x of size %d", addr, data.size()};
        }

        for (const auto value : data) {
            const auto realAddr = addr & 0xFFFFFF;
            if (value != 0 || Values_.contains(realAddr)) {
                Values_[realAddr] = value;
            }
            addr++;
        }
        return std::nullopt;
    }

    TRamSnapshot GetRamSnapshot() const {
        return Values_;
    }

private:
    TRamSnapshot Values_;
};

std::string DumpRamSnapshot(const TRamSnapshot& ram) {
    std::stringstream ss;
    for (const auto& pair : ram) {
        ss << "[" << pair.first << "] = " << static_cast<int>(pair.second) << "\n";
    }
    return ss.str();
}

TRamSnapshot GetRamSnapshot(const json& ram) {
    TRamSnapshot res;
    for (const auto& pair : ram) {
        const auto addr = pair[0].get<int>();
        const auto value = pair[1].get<TByte>();
        if (value != 0) {
            res[addr] = value;
        }
    }
    return res;
}

std::vector<std::pair<TAddressType, TByte>> GetRamDiff(const TRamSnapshot& ram0, const TRamSnapshot& ram1) {
    std::vector<std::pair<TAddressType, TByte>> diff;
    for (const auto& p : ram1) {
        const auto it = ram0.find(p.first);
        if (it == ram0.end() || it->second != p.second) {
            diff.emplace_back(p.first, p.second);
        }
    }
    for (const auto& p : ram0) {
        if (ram1.find(p.first) == ram1.end()) {
            diff.emplace_back(p.first, 0);
        }
    }
    if (!diff.empty()) std::sort(diff.begin(), diff.end());
    return diff;
}

json LoadTestFile(std::string_view path) {
    std::ifstream f(path.data());
    json data = json::parse(f);
    ferr << "\"" << path << "\" parsed" << std::endl;
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
    if ((lhs.SR ^ rhs.SR) & 0b1111'0111'0001'1111) diffs.emplace_back("SR");

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

    auto initRegs = ParseRegisters(initialJson);
    auto expectedRegs = ParseRegisters(finalJson);
    auto actualRegs = initRegs;

    TTestDevice device{initRegs.PC, initialJson["prefetch"], initialJson["ram"]};
    const auto actualRam0 = device.GetRamSnapshot();
    const auto expectedRam0 = GetRamSnapshot(initialJson["ram"]);

    NEmulator::TContext ctx{.Registers = actualRegs, .Memory = device};
    const auto err = NEmulator::Emulate(ctx);

    if (err) {
        ferr << "Got error: " << err->GetWhat() << std::endl;

        // this program counter means there really was an illegal instruction
        return (expectedRegs.PC == 0x1400);
    }

    const auto actualRam1 = device.GetRamSnapshot();
    const auto expectedRam1 = GetRamSnapshot(finalJson["ram"]);

    const auto regsDiff = DumpDiff(expectedRegs, actualRegs);
    const bool ramDiffers = GetRamDiff(actualRam0, actualRam1) != GetRamDiff(expectedRam0, expectedRam1);

    if (regsDiff || ramDiffers) {
        ferr << "Test name: \"" << test["name"].get<std::string>() << "\"" << std::endl << std::endl;

        if (regsDiff) {
            ferr << "Initial registers:" << std::endl;
            ferr << Dump(initRegs) << std::endl;

            ferr << "Actual final registers:" << std::endl;
            ferr << Dump(actualRegs) << std::endl;

            ferr << "Expected final registers:" << std::endl;
            ferr << Dump(expectedRegs) << std::endl;

            ferr << "Differing registers: " << *regsDiff << std::endl << std::endl;
        }

        if (ramDiffers) {
            ferr << "Initial RAM:" << std::endl;
            ferr << DumpRamSnapshot(actualRam0) << std::endl;

            ferr << "Actual RAM:" << std::endl;
            ferr << DumpRamSnapshot(actualRam1) << std::endl;

            ferr << "Expected RAM:" << std::endl;
            ferr << DumpRamSnapshot(expectedRam1) << std::endl;

            ferr << "RAM differs" << std::endl;
        }

        return false;
    }

    return true;
}

bool WorkOnFile(const json& file) {
    std::size_t size = file.size();
    ferr << "work on file with " << size << " tests" << std::endl;

    int passed = 0;
    int failed = 0;
    int ignored = size;

    for (std::size_t i = 0; i < size; ++i) {
        const bool ok = WorkOnTest(file[i]);
        ferr << (i + 1) << "/" << size << " test is " << (ok ? "OK" : "FAIL") << std::endl;

        --ignored;
        if (ok) {
            ++passed;
        } else {
            ++failed;
            //break;
        }
    }
    ferr << "TOTAL TESTS: " << size << std::endl;
    ferr << "PASSED TESTS: " << passed << std::endl;
    ferr << "FAILED TESTS: " << failed << std::endl;
    ferr << "IGNORED TESTS: " << ignored << std::endl;
    return passed == size;
}

} // namespace

int main() {
    namespace fs = std::filesystem;

    std::vector<std::string> paths;
    for (const auto& entry : fs::directory_iterator("/home/mango/ProcessorTests/680x0/68000/v1")) {
        auto path = entry.path().string();
        if (!path.ends_with(".json")) {
            continue;
        }
        paths.emplace_back(std::move(path));
    }
    std::sort(paths.begin(), paths.end(), [](auto&& lhs, auto&& rhs) {
        for (std::size_t i = 0; i < std::min(lhs.size(), rhs.size()); ++i) {
            if (std::tolower(lhs[i]) < std::tolower(rhs[i])) return true;
            if (std::tolower(lhs[i]) > std::tolower(rhs[i])) return false;
        }
        return lhs.size() < rhs.size();
    });

    const auto shouldRunTest = [](int index) {
        if (index >= 1 && index <= 26) return true;
        if (index >= 28 && index <= 35) return true;
        if (index >= 39 && index <= 43) return true;
        if (index >= 51 && index <= 64) return true;
        if (index >= 69 && index <= 71) return true;
        if (index >= 75 && index <= 89) return true;
        if (index >= 92 && index <= 97) return true;
        if (index >= 109 && index <= 116) return true;
        return false;
    };

    constexpr int threadCount = 10;
    std::mutex mut;
    int curIndex = 1;
    int totalCount = 0;

    fs::remove_all("logs");
    fs::create_directories("logs");

    std::vector<std::thread> threads;
    for (int i = 0; i < threadCount; ++i) {
        threads.emplace_back([&]() {
            while (true) {
                std::string path;
                {
                    std::lock_guard guard{mut};
                    while (curIndex <= paths.size() && !shouldRunTest(curIndex)) {
                        ++curIndex;
                    }
                    if (curIndex > paths.size()) {
                        return;
                    }
                    path = paths[curIndex - 1];
                    std::cerr << "working of file " << path.substr(path.rfind('/') + 1) << std::endl;
                    ++curIndex;
                    ++totalCount;
                }

                std::string part = path.substr(path.rfind('/') + 1);
                part = part.substr(0, part.rfind('.'));
                ferr = std::ofstream{"logs/" + part};

                auto file = LoadTestFile(path);
                WorkOnFile(file);
            }
        });
    }
    for (auto& t : threads) {
        t.join();
    }
    std::cerr << "Total file count: " << totalCount << std::endl;
}
