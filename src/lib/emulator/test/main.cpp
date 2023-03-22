#include <fstream>
#include <iostream>

#include <lib/emulator/emulator.h>

#include <thirdparty/include/nlohmann/json.hpp>

using json = nlohmann::json;

namespace {

class TTestDevice : public NMemory::IDevice {
public:
    TTestDevice(int pc, const json& prefetch, const json& ram) {
        // fill RAM
        for (const auto& pair : ram) {
            const auto addr = pair[0].get<int>();
            const auto value = pair[1].get<std::byte>();
            Values_[addr] = value;
        }

        // fill prefetch
        if (prefetch.size() != 2) {
            throw std::runtime_error("why not 2 items?");
        }
        Values_[pc] = static_cast<std::byte>(prefetch[0].get<int>() >> 8);
        Values_[pc + 1] = static_cast<std::byte>(prefetch[0].get<int>() % 256);
        Values_[pc + 2] = static_cast<std::byte>(prefetch[1].get<int>() >> 8);
        Values_[pc + 3] = static_cast<std::byte>(prefetch[1].get<int>() % 256);
    }

    NMemory::TDataHolder Read(NMemory::TAddressType addr, NMemory::TAddressType size) override {
        NMemory::TDataHolder res;
        for (int i = addr; i < addr + size; ++i) {
            if (const auto it = Values_.find(addr); it != Values_.end()) {
                res.push_back(it->second);
            } else {
                res.push_back(static_cast<std::byte>(0));
            }
        }
        return res;
    }

    void Write(NMemory::TAddressType addr, NMemory::TDataView data) override {
        for (const auto value : data) {
            Values_[addr++] = value;
        }
    }

private:
    std::map<int, std::byte> Values_;
};

json LoadTestFile(std::string_view path) {
    std::ifstream f(path.data());
    json data = json::parse(f);
    std::cerr << "\"" << path << "\" parsed" << std::endl;
    return data;
}

bool Equals(const NRegisters::TRegisters& lhs, const NRegisters::TRegisters& rhs) {
    for (int i = 0; i < 8; ++i) {
        if (lhs.D[i] != rhs.D[i]) return false;
    }
    for (int i = 0; i < 7; ++i) {
        if (lhs.A[i] != rhs.A[i]) return false;
    }
    if (lhs.USP != rhs.USP) return false;
    if (lhs.SSP != rhs.SSP) return false;
    if (lhs.PC != rhs.PC) return false;
    if (lhs.SR != rhs.SR) return false;
    return true;
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
    if (!Equals(expectedFinal, actualFinal)) {
        std::cerr << "Initial registers:" << std::endl;
        std::cerr << Dump(initial) << std::endl;

        std::cerr << "Actual final registers:" << std::endl;
        std::cerr << Dump(actualFinal) << std::endl;

        std::cerr << "Expected final registers:" << std::endl;
        std::cerr << Dump(expectedFinal) << std::endl;

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
    auto file = LoadTestFile("/home/mango/proctest/NOP.json");
    WorkOnFile(file);
}
