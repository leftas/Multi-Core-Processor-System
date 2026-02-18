/*
 * File: assignment1.cpp
 *
 * Framework to implement Task 1 of the Multi-Core Processor Systems lab
 * session. This uses the framework library to interface with tracefiles which
 * will drive the read/write requests
 *
 * Author(s): Michiel W. van Tol, Mike Lankamp, Jony Zhang,
 *            Konstantinos Bousias, Simon Polstra
 *
 */

#include <algorithm>
#include <iostream>
#include <optional>
#include <systemc>
#include <cmath>
#include <array>
#include <list>
#include "psa.h"
#include "sysc/communication/sc_signal_rv_ports.h"

using namespace std;
using namespace sc_core; // This pollutes namespace, better: only import what you need.
using ADDRESS_UNIT = uint8_t;

static constexpr size_t MEM_SIZE = 8912;
static constexpr size_t CACHE_SETS = 128;
static constexpr size_t CACHE_LINE_SIZE = 32;
static constexpr size_t CACHE_WAYS = 8;

static constexpr size_t CACHE_SIZE = CACHE_SETS * CACHE_WAYS * CACHE_LINE_SIZE;

static constexpr size_t OFFSET_BITS = std::log2(CACHE_LINE_SIZE / sizeof(ADDRESS_UNIT)); // 5
static constexpr size_t INDEX_BITS = std::log2(CACHE_SETS); // 7

static_assert(CACHE_SIZE % (CACHE_LINE_SIZE * CACHE_WAYS) == 0,
              "Cache size must be a multiple of cache line size * cache ways");


SC_MODULE(Memory) {
    public:
    enum Function { FUNC_READ, FUNC_WRITE };

    enum RetCode { RET_READ_DONE, RET_WRITE_DONE };

    sc_in<bool> Port_CLK;
    sc_in<Function> Port_Func;
    sc_in<uint64_t> Port_Addr;
    sc_out<RetCode> Port_Done;
    sc_inout_rv<sizeof(ADDRESS_UNIT) * 32> Port_Data;

    SC_CTOR(Memory) {
        SC_THREAD(execute);
        sensitive << Port_CLK.pos();
        dont_initialize();

        m_data = new ADDRESS_UNIT[MEM_SIZE];
    }

    ~Memory() {
        delete[] m_data;
    }

    private:
    ADDRESS_UNIT *m_data;

    void execute() {
        while (true) {
            wait(Port_Func.value_changed_event());

            Function f = Port_Func.read();
            uint64_t addr = Port_Addr.read();
            uint32_t data = 0;
            if (f == FUNC_WRITE) {
                data = Port_Data.read().to_uint();
                log(name(), "received write on address", addr, "with data", data);
            } else {
                log(name(), "received read on address", addr);
            }

            // This simulates memory read/write delay
            wait(100);

            if (f == FUNC_READ) {
                Port_Data.write((addr < MEM_SIZE) ? m_data[addr] : 0);
                Port_Done.write(RET_READ_DONE);
                wait();
                Port_Data.write(float_64_bit_wire); // string with 64 "Z"'s
            } else {
                if (addr < MEM_SIZE) {
                    m_data[addr] = data;
                }
                Port_Done.write(RET_WRITE_DONE);
            }
        }
    }
};

SC_MODULE(Bus) {
public :
    sc_in<bool> Port_CLK;
    sc_inout_rv<64> Port_BusAddr;
    sc_inout_rv<2> Port_BusFunc;
    sc_inout_rv<1> Port_BusDone;
    sc_inout_rv<sizeof(ADDRESS_UNIT)*32> Port_BusData;
    sc_out<Memory::Function> Port_MemFunc;
    sc_in<Memory::RetCode> Port_MemDone;
    sc_out<uint64_t> Port_MemAddr;
    sc_inout_rv<sizeof(ADDRESS_UNIT) * 32> Port_MemData;
public :
    SC_CTOR ( Bus )
    {
    }
    virtual bool read(uint64_t addr)
    {
        Port_BusAddr.write(addr);
        return true;
    };
    virtual bool write (uint64_t addr, uint64_t data)
    {
        Port_BusAddr.write(addr);
        return true;
    }
};

struct Cacheline {
    size_t _idx;
    size_t tag = 0;
    bool valid = false;
    bool dirty = false;
    array<uint32_t, CACHE_LINE_SIZE / sizeof(ADDRESS_UNIT)> data {};
};

struct Cacheset {
    list<Cacheline> lines {};

    Cacheset(){
        for(size_t way =0; way<CACHE_WAYS; ++way){
            Cacheline line{way};
            lines.emplace_back(line);
        }
    }

    void touch(Cacheline& way)
    {
        auto iter = find_if(lines.begin(), lines.end(), [&](Cacheline& e){return &e == &way;});
        if(iter == lines.end())
            return;
        lines.splice(lines.begin(), lines, iter);
    }
    Cacheline& evict()
    {
        return lines.back();
    }

};

class Cache: public sc_module {
    public:
    sc_in<bool> Port_CLK;

    sc_in<Memory::Function> Port_Func;
    sc_out<Memory::RetCode> Port_Done;

    sc_in<uint64_t> Port_Addr;
    sc_inout_rv<sizeof(ADDRESS_UNIT) * 32> Port_Data;

    sc_inout_rv<2> Port_MemFunc;
    sc_inout_rv<1> Port_MemDone;

    sc_inout_rv<64> Port_MemAddr;
    sc_inout_rv<sizeof(ADDRESS_UNIT) * 32> Port_MemData;

   Cache(sc_module_name name_, int id_): sc_module(name_), id(id_) {
        SC_THREAD(execute);
        sensitive << Port_CLK.pos();
        log(name(), "constructed with id", id);
        dont_initialize();

    }

private:
    int id;
    array<Cacheset, CACHE_SETS> m_cache;

    void write_out_read(ADDRESS_UNIT data)
    {
        Port_Data.write(data);
        Port_Done.write(Memory::RET_READ_DONE);
        wait();
        Port_Data.write(float_64_bit_wire); // string with 64 "Z"'s
    }

    void execute()
    {
        while (true) {
            wait(Port_Func.value_changed_event());

            Memory::Function f = Port_Func.read();
            uint64_t addr = Port_Addr.read();
            std::optional<uint32_t> result; // result will be gone if we not gonna take it instantly.

            if (f == Memory::FUNC_WRITE)
                result = Port_Data.read().to_uint();

            size_t offset = addr & ((1 << OFFSET_BITS) - 1); // offset bitmask -> indicates which offset in cacheline we select.
            size_t index  = (addr >> OFFSET_BITS) & ((1 << INDEX_BITS) - 1); // index bitmask -> indicates which cache set we select.
            size_t tag    = addr >> (OFFSET_BITS + INDEX_BITS); // leftovers for tag -> matching the cacheline itself!

            auto& current_set = m_cache[index];

            if (f == Memory::FUNC_READ)
                log(name(), "read address =", addr);
            if (f == Memory::FUNC_WRITE)
                log(name(), "write address =", addr);

            wait(1);

            bool found = false;
            for (Cacheline& way : current_set.lines) {
                if (way.valid && way.tag == tag) {
                    // fast path
                    current_set.touch(way);
                    if (f == Memory::FUNC_READ) {
                        log(name(), "read hit address =", addr, "set =", index, "line =", way._idx);
                        // touch line to make sure it's recently used.
                        write_out_read(way.data[offset]);
                        stats_readhit(0);
                    }
                    if (f == Memory::FUNC_WRITE) {
                        log(name(), "write hit address =", addr, "set =", index, "line =", way._idx);
                        way.data[offset] = result.value();
                        way.dirty = true;
                        Port_Done.write(Memory::RET_WRITE_DONE);
                        stats_writehit(0);
                    }
                    found = true;
                    break;
                }
            }

            if (found)
                continue;

            if (f == Memory::FUNC_READ) {
                stats_readmiss(0);
                log(name(), "read miss address =", addr);
            } else {
                stats_writemiss(0);
                log(name(), "write miss address =", addr);
            }

            // Taking a slow path. Accessing memory

            Port_MemAddr.write(addr);
            Port_MemFunc.write(Memory::FUNC_READ);
            wait(Port_MemDone.value_changed_event());

            if (f == Memory::FUNC_READ) 
                result = Port_MemData.read().to_uint();

            Cacheline* assign_way = nullptr;

            for (Cacheline& way : current_set.lines) {
                if (!way.valid) {
                    assign_way = &way;
                    break;
                }
            }

            if (assign_way == nullptr) {
                // Lack of space in the cacheset.
                // Evict the last one.
                assign_way = &current_set.evict();

                uint64_t victim_line_addr = assign_way->tag << (INDEX_BITS + OFFSET_BITS) | (index << OFFSET_BITS);
                if (assign_way->dirty) {
                    log(name(), "evict dirty line address =", victim_line_addr, "set =", index, "line =", assign_way->_idx);
                    Port_MemAddr.write(victim_line_addr);
                    Port_MemData.write(assign_way->data[offset]);
                    wait();
                    Port_MemFunc.write(Memory::FUNC_WRITE);
                    wait(Port_MemDone.value_changed_event());
                    Port_MemData.write(float_64_bit_wire);
                } else {
                    log(name(), "evict clean line address =", victim_line_addr, "set =", index, "line =", assign_way->_idx);
                }

            }

            // Overwrite and put it to the front
            assign_way->tag = tag;
            assign_way->data[offset] = result.value();
            assign_way->valid = true;
            assign_way->dirty = (f == Memory::FUNC_WRITE);
            current_set.touch(*assign_way);

            log(name(), "write completed address =", addr, "set =", index, "line =", assign_way->_idx);

            if (f == Memory::FUNC_READ) {
                write_out_read(result.value());
                log(name(), "read done address =", addr);
            } else {
                Port_Done.write(Memory::RET_WRITE_DONE);
                log(name(), "write done address =", addr);
            }
        }
    }
};

class CPU : public sc_module{
    public:
    sc_in<bool> Port_CLK;
    sc_in<Memory::RetCode> Port_MemDone;
    sc_out<Memory::Function> Port_MemFunc;
    sc_out<uint64_t> Port_MemAddr;
    sc_inout_rv<sizeof(ADDRESS_UNIT) * 32> Port_MemData;

    CPU(sc_module_name name_, int id_): sc_module(name_), id(id_) {
        SC_THREAD(execute);
        sensitive << Port_CLK.pos();
        log(name(), "constructed with id", id);
        dont_initialize();
    }

    private:
    int id;
    void execute() {
        TraceFile::Entry tr_data;
        Memory::Function f;

        // Loop until end of tracefile
        while (!tracefile_ptr->eof()) {
            // Get the next action for the processor in the trace
            if (!tracefile_ptr->next(0, tr_data)) {
                cerr << "Error reading trace for CPU" << endl;
                break;
            }

            switch (tr_data.type) {
            case TraceFile::ENTRY_TYPE_READ:
                f = Memory::FUNC_READ;
                break;

            case TraceFile::ENTRY_TYPE_WRITE:
                f = Memory::FUNC_WRITE;
                break;

            case TraceFile::ENTRY_TYPE_NOP: break;

            default:
                cerr << "Error, got invalid data from Trace" << endl;
                exit(0);
            }

            if (tr_data.type != TraceFile::ENTRY_TYPE_NOP) {
                Port_MemAddr.write(tr_data.addr);
                Port_MemFunc.write(f);

                if (f == Memory::FUNC_WRITE) {
                    // No data in trace, use address * 10 as data value.
                    ADDRESS_UNIT data = tr_data.addr * 10;
                    log(name(), "write value", data,
                            "to address", tr_data.addr);
                    Port_MemData.write(data);
                    wait();
                    // Now float the data wires with 64 "Z"'s
                    Port_MemData.write(float_64_bit_wire);

                } else {
                    log(name(), "read on address", tr_data.addr);
                }

                wait(Port_MemDone.value_changed_event());

                if (f == Memory::FUNC_READ) {
                    log(name(), "read data", Port_MemData.read().to_uint(),
                            "from address", tr_data.addr);
                }
            } else {
                log(name(), "executing NOP");
            }
            // Advance one cycle in simulated time
            wait();
        }

        // Finished the Tracefile, now stop the simulation
        sc_stop();
    }
};


int sc_main(int argc, char *argv[]) {
    sc_report_handler::set_verbosity_level(SC_MEDIUM);
    // Uncomment the next line to silence the log() messages.
    // sc_report_handler::set_verbosity_level(SC_LOW);
    sc_report_handler::set_actions(SC_ID_VECTOR_CONTAINS_LOGIC_VALUE_, SC_ABORT);

    try {
        // Get the tracefile argument and create Tracefile object
        // This function sets tracefile_ptr and num_cpus
        init_tracefile(&argc, &argv);

        // init_tracefile changed argc and argv so we cannot use
        // getopt anymore. "-q" must be specified after the tracefile.
        if (argc == 2 && !strcmp(argv[0], "-q")) {
            sc_report_handler::set_verbosity_level(SC_LOW);
        }

        // Initialize statistics counters
        stats_init();

        // The clock that will drive the CPU and Memory
        sc_clock clk;
        // Instantiate Modules
        Memory mem("memory");
        Bus bus("bus");
        vector<CPU*> cpus(num_cpus);
        vector<Cache*> caches(num_cpus);

        // Signals
        sc_buffer<Memory::Function> sigMemFunc;
        sc_buffer<Memory::RetCode> sigMemDone;
        sc_signal<uint64_t> sigMemAddr;
        sc_signal_rv<sizeof(ADDRESS_UNIT) * 32> sigMemData;
        sc_signal_rv<64> sigBusAddr;
        sc_signal_rv<sizeof(ADDRESS_UNIT)*32> sigBusData;
        sc_signal_rv<2> sigBusFunc;
        sc_signal_rv<1> sigBusDone;

        bus.Port_MemFunc(sigMemFunc);
        bus.Port_MemDone(sigMemDone);
        bus.Port_MemAddr(sigMemAddr);
        bus.Port_MemData(sigMemData);

        bus.Port_BusAddr(sigBusAddr);
        bus.Port_BusData(sigBusData);
        bus.Port_BusFunc(sigBusFunc);
        bus.Port_BusDone(sigBusDone);

        mem.Port_Func(sigMemFunc);
        mem.Port_Done(sigMemDone);
        mem.Port_Addr(sigMemAddr);
        mem.Port_Data(sigMemData);

        bus.Port_CLK(clk);
        mem.Port_CLK(clk);

        vector<sc_buffer<Memory::Function>*> cpu_funcs(num_cpus);
        vector<sc_buffer<Memory::RetCode>*>  cpu_dones(num_cpus);
        vector<sc_signal<uint64_t>*>         cpu_addrs(num_cpus);
        vector<sc_signal_rv<sizeof(ADDRESS_UNIT) * 32>*> cpu_datas(num_cpus);

        for (uint32_t i = 0; i < num_cpus; ++i) {
            string cpu_name = "cpu_" + to_string(i);
            string cache_name = "cache_" + to_string(i);

            cpus[i] = new CPU(cpu_name.c_str(), i);
            caches[i] = new Cache(cache_name.c_str(), i);

            cpu_funcs[i] = new sc_buffer<Memory::Function>();
            cpu_dones[i] = new sc_buffer<Memory::RetCode>();
            cpu_addrs[i] = new sc_signal<uint64_t>();
            cpu_datas[i] = new sc_signal_rv<sizeof(ADDRESS_UNIT) * 32>();

            // Connect CPU <-> Cache
            cpus[i]->Port_MemFunc(*cpu_funcs[i]);
            cpus[i]->Port_MemDone(*cpu_dones[i]);
            cpus[i]->Port_MemAddr(*cpu_addrs[i]);
            cpus[i]->Port_MemData(*cpu_datas[i]);
            cpus[i]->Port_CLK(clk);

            caches[i]->Port_Func(*cpu_funcs[i]);
            caches[i]->Port_Done(*cpu_dones[i]);
            caches[i]->Port_Addr(*cpu_addrs[i]);
            caches[i]->Port_Data(*cpu_datas[i]);

            caches[i]->Port_MemAddr(sigBusAddr); 
            caches[i]->Port_MemData(sigBusData);
            caches[i]->Port_MemFunc(sigBusFunc);
            caches[i]->Port_MemDone(sigBusDone);
            caches[i]->Port_CLK(clk);
            }

        cout << "Running (press CTRL+C to interrupt)... " << endl;


        // Start Simulation
        sc_start();

        // Print statistics after simulation finished
        stats_print();
        // mem.dump(); // Uncomment to dump memory to stdout.
    }

    catch (exception &e) {
        cerr << e.what() << endl;
    }

    return 0;
}
