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

using namespace std;
using namespace sc_core; // This pollutes namespace, better: only import what you need.
using ADDRESS_UNIT = uint64_t;

static constexpr size_t MEM_SIZE = 8912;
static constexpr size_t CACHE_SIZE = 32 * 1024;
static constexpr size_t CACHE_LINE_SIZE = 32;
static constexpr size_t CACHE_WAYS = 8;
static constexpr size_t CACHE_SETS = CACHE_SIZE / (CACHE_LINE_SIZE * CACHE_WAYS);

static constexpr size_t OFFSET_BITS = std::log2(CACHE_LINE_SIZE / sizeof(ADDRESS_UNIT));
static constexpr size_t INDEX_BITS = std::log2(CACHE_SETS);

static_assert(CACHE_SIZE % (CACHE_LINE_SIZE * CACHE_WAYS) == 0,
              "Cache size must be a multiple of cache line size * cache ways");


SC_MODULE(Memory) {
    public:
    enum Function { FUNC_READ, FUNC_WRITE };

    enum RetCode { RET_READ_DONE, RET_WRITE_DONE };

    sc_in<bool> Port_CLK;
    sc_in<Function> Port_Func;
    sc_in<ADDRESS_UNIT> Port_Addr;
    sc_out<RetCode> Port_Done;
    sc_inout_rv<sizeof(ADDRESS_UNIT) * 8> Port_Data;

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
            ADDRESS_UNIT addr = Port_Addr.read();
            ADDRESS_UNIT data = 0;
            if (f == FUNC_WRITE) {
                data = Port_Data.read().to_uint64();
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

struct Cacheline {
    size_t _idx;
    size_t tag = 0;
    bool valid = false;
    bool dirty = false;
    array<ADDRESS_UNIT, CACHE_LINE_SIZE / sizeof(ADDRESS_UNIT)> data {};
};

struct Cacheset {
    list<Cacheline> lines {};

    Cacheset(){
        for(size_t way =0; way<CACHE_WAYS; ++way){
            Cacheline line{._idx = way};
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

SC_MODULE(Cache) {
    public:
    sc_in<bool> Port_CLK;

    sc_in<Memory::Function> Port_Func;
    sc_out<Memory::RetCode> Port_Done;

    sc_in<ADDRESS_UNIT> Port_Addr;
    sc_inout_rv<sizeof(ADDRESS_UNIT) * 8> Port_Data;

    sc_out<Memory::Function> Port_MemFunc;
    sc_in<Memory::RetCode> Port_MemDone;

    sc_out<ADDRESS_UNIT> Port_MemAddr;
    sc_inout_rv<sizeof(ADDRESS_UNIT) * 8> Port_MemData;

    SC_CTOR(Cache) {
        SC_THREAD(execute);
        sensitive << Port_CLK.pos();
        dont_initialize();

    }

private:
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
            ADDRESS_UNIT addr = Port_Addr.read();
            std::optional<ADDRESS_UNIT> result; // result will be gone if we not gonna take it instantly.

            if (f == Memory::FUNC_WRITE)
                result = Port_Data.read().to_uint64();

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
                        log(name(), "read hit address =", addr, "set =", index, "line =", offset);
                        // touch line to make sure it's recently used.
                        write_out_read(way.data[offset / sizeof(ADDRESS_UNIT)]);
                    }
                    if (f == Memory::FUNC_WRITE) {
                        log(name(), "write hit address =", addr, "set =", index, "line =", offset);
                        way.data[offset / sizeof(ADDRESS_UNIT)] = result.value();
                        way.dirty = true;
                        Port_Done.write(Memory::RET_WRITE_DONE);
                    }
                    found = true;
                    break;
                }
            }

            if (found)
                continue;

            // Taking a slow path. Accessing memory

            Port_MemAddr.write(addr);
            // NOTE: This is not entirely correct. We need to fill the whole cache line here.
            Port_MemFunc.write(Memory::FUNC_READ);
            wait(Port_MemDone.value_changed_event());

            if (f == Memory::FUNC_READ)
                result = Port_MemData.read().to_uint64();

            bool inserted = false;

            for (Cacheline& way : current_set.lines) {
                if (!way.valid) {
                    way.tag = tag;
                    way.data[offset / sizeof(ADDRESS_UNIT)] = result.value();
                    way.valid = true;
                    way.dirty = (f == Memory::FUNC_WRITE) ? true:false;
                    inserted = true;
                    break;
                }
            }

            if (!inserted) {
                // Lack of space in the cacheset.
                // Evict the last one.
                Cacheline& victim_line = current_set.evict();
                if (victim_line.dirty)
                {
                    ADDRESS_UNIT victim_line_addr = victim_line.tag << (INDEX_BITS + OFFSET_BITS) | (index << OFFSET_BITS);
                    log(name(), "evict dirty line address =", victim_line_addr, "set =", index, "line =", victim_line.tag);
                    wait();
                    Port_MemAddr.write(victim_line_addr);
                    Port_MemData.write(victim_line.data[offset / sizeof(ADDRESS_UNIT)]);
                    Port_MemFunc.write(Memory::FUNC_WRITE);
                    wait(Port_MemDone.value_changed_event());
                }
                else
                {
                    log(name(), "evict clean line address =", victim_line.tag, "set =", index, "line =", victim_line.tag);
                }
                // Overwrite and put it to the front
                victim_line.tag = tag;
                victim_line.data[offset / sizeof(ADDRESS_UNIT)] = result.value();
                victim_line.valid = true;
                victim_line.dirty = false;
                current_set.touch(victim_line);
            }

            if (f == Memory::FUNC_READ) 
                write_out_read(result.value());
            else
                Port_Done.write(Memory::RET_WRITE_DONE);
        }
    }
};

SC_MODULE(CPU) {
    public:
    sc_in<bool> Port_CLK;
    sc_in<Memory::RetCode> Port_MemDone;
    sc_out<Memory::Function> Port_MemFunc;
    sc_out<ADDRESS_UNIT> Port_MemAddr;
    sc_inout_rv<sizeof(ADDRESS_UNIT) * 8> Port_MemData;

    SC_CTOR(CPU) {
        SC_THREAD(execute);
        sensitive << Port_CLK.pos();
        dont_initialize();
    }

    private:
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

            // To demonstrate the statistic functions, we generate a 50%
            // probability of a 'hit' or 'miss', and call the statistic
            // functions below
            int j = rand() % 2;

            switch (tr_data.type) {
            case TraceFile::ENTRY_TYPE_READ:
                f = Memory::FUNC_READ;
                if (j)
                    stats_readhit(0);
                else
                    stats_readmiss(0);
                break;

            case TraceFile::ENTRY_TYPE_WRITE:
                f = Memory::FUNC_WRITE;
                if (j)
                    stats_writehit(0);
                else
                    stats_writemiss(0);
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
                    log(name(), "read data", Port_MemData.read().to_uint64(),
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

        // Initialize statistics counters
        stats_init();

        // Instantiate Modules
        Memory mem("memory");
        CPU cpu("cpu");
        Cache cache("cache");

        // Signals
        sc_buffer<Memory::Function> sigMemFunc;
        sc_buffer<Memory::RetCode> sigMemDone;
        sc_signal<ADDRESS_UNIT> sigMemAddr;
        sc_signal_rv<sizeof(ADDRESS_UNIT) * 8> sigMemData;

        sc_buffer<Memory::Function> sigCacheFunc;
        sc_buffer<Memory::RetCode> sigCacheDone;
        sc_signal<ADDRESS_UNIT> sigCacheAddr;
        sc_signal_rv<sizeof(ADDRESS_UNIT) * 8> sigCacheData;

        // The clock that will drive the CPU and Memory
        sc_clock clk;

        // Connecting module ports with signals

        cache.Port_MemFunc(sigCacheFunc);
        cache.Port_MemAddr(sigCacheAddr);
        cache.Port_MemData(sigCacheData);
        cache.Port_MemDone(sigCacheDone);

        mem.Port_Func(sigCacheFunc);
        mem.Port_Addr(sigCacheAddr);
        mem.Port_Data(sigCacheData);
        mem.Port_Done(sigCacheDone);

        cache.Port_Func(sigMemFunc);
        cache.Port_Addr(sigMemAddr);
        cache.Port_Data(sigMemData);
        cache.Port_Done(sigMemDone);

        cpu.Port_MemFunc(sigMemFunc);
        cpu.Port_MemAddr(sigMemAddr);
        cpu.Port_MemData(sigMemData);
        cpu.Port_MemDone(sigMemDone);

        mem.Port_CLK(clk);
        cpu.Port_CLK(clk);
        cache.Port_CLK(clk);

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
