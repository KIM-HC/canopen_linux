#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/timer.hpp>       // for system timer
#include <lely/io2/sys/sigset.hpp>      // for clean shutdown when ctrl  c is pressed
#include <lely/coapp/fiber_driver.hpp>  // for making driver
#include <lely/coapp/master.hpp>        // for canopen master


#include <iostream>

using namespace std::chrono_literals;
// using namespace lely;

// This driver inherits from FiberDriver, which means that all CANopen event
// callbacks, such as OnBoot, run as a task inside a "fiber" (or stackful
// coroutine).
class MyDriver : public lely::canopen::FiberDriver {
    public:
    using FiberDriver::FiberDriver;

    private:
    // This function gets called when the boot-up process of the slave completes.
    // The 'st' parameter contains the last known NMT state of the slave
    // (typically pre-operational), 'es' the error code (0 on success), and 'what'
    // a description of the error, if any.
    void OnBoot(lely::canopen::NmtState /*st*/, char es,
            const std::string& what) noexcept override {
        if (!es || es == 'L') {
            std::cout << "slave " << static_cast<int>(id()) << " booted sucessfully"
                        << std::endl;
        } else {
            std::cout << "slave " << static_cast<int>(id())
                        << " failed to boot: " << what << std::endl;
        }   
    }

    // This function gets called during the boot-up process for the slave. The
    // 'res' parameter is the function that MUST be invoked when the configuration
    // is complete. Because this function runs as a task inside a coroutine, it
    // can suspend itself and wait for an asynchronous function, such as an SDO
    // request, to complete.
    void OnConfig(std::function<void(std::error_code ec)> res) noexcept override {
        try {
            // Perform a few SDO write requests to configure the slave. The
            // AsyncWrite() function returns a future which becomes ready once the
            // request completes, and the Wait() function suspends the coroutine for
            // this task until the future is ready.

            // Configure the slave to monitor the heartbeat of the master (node-ID 99)
            // with a timeout of 2000 ms.
            // Wait(AsyncWrite<uint32_t>(0x1016, 1, (1 << 16) | 2000));
            Wait(AsyncWrite<uint32_t>(0x1016, 1, 2000));
            // Configure the slave to produce a heartbeat every 1000 ms.
            Wait(AsyncWrite<uint16_t>(0x1017, 0, 1000));
            // Configure the heartbeat consumer on the master.
            ConfigHeartbeat(2000ms);

            // Report success (empty error code).
            res({});
        } catch (lely::canopen::SdoError& e) {
            // If one of the SDO requests resulted in an error, abort the
            // configuration and report the error code.
            res(e.code());
        }
    }

    // This function is similar to OnConfg(), but it gets called by the
    // AsyncDeconfig() method of the master.
    void OnDeconfig(std::function<void(std::error_code ec)> res) noexcept override {
        try {
            // Disable the heartbeat consumer on the master.
            ConfigHeartbeat(0ms);
            // Disable the heartbeat producer on the slave.
            Wait(AsyncWrite<uint16_t>(0x1017, 0, 0));
            // Disable the heartbeat consumer on the slave.
            Wait(AsyncWrite<uint32_t>(0x1016, 1, 0));
            res({});
        } catch (lely::canopen::SdoError& e) {
            res(e.code());
        }
    }

    // This function gets called every time a value is written to the local object
    // dictionary of the master by an RPDO (or SDO, but that is unlikely for a
    // master), *and* the object has a known mapping to an object on the slave for
    // which this class is the driver. The 'idx' and 'subidx' parameters are the
    // object index and sub-index of the object on the slave, not the local object
    // dictionary of the master.
    void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override {
        std::cout << "rpdo idx: " << idx << std::endl;
        if (idx == 0x4001 && subidx == 0) {
            // Obtain the value sent by PDO from object 4001:00 on the slave.
            uint32_t val = rpdo_mapped[0x4001][0];
            // Increment the value and store it to an object in the local object
            // dictionary that will be sent by TPDO to object 4000:00 on the slave.
            tpdo_mapped[0x4000][0] = ++val;
        }
    }

};

int main() {
    // Initialize the I/O library. This is required on Windows, but a no-op on
    // Linux (for now).
    lely::io::IoGuard io_guard;

    // Create an I/O context to synchronize I/O services during shutdown.
    lely::io::Context ctx;
    // Create an platform-specific I/O polling instance to monitor the CAN bus, as
    // well as timers and signals.
    lely::io::Poll poll(ctx);
    // Create a polling event loop and pass it the platform-independent polling
    // interface. If no tasks are pending, the event loop will poll for I/O
    // events.
    lely::ev::Loop loop(poll.get_poll());
    // I/O devices only need access to the executor interface of the event loop.
    auto exec = loop.get_executor();
    // Create a timer using a monotonic clock, i.e., a clock that is not affected
    // by discontinuous jumps in the system time.
    lely::io::Timer timer(poll, exec, CLOCK_MONOTONIC);
    // Create a virtual SocketCAN CAN controller and channel, and do not modify
    // the current CAN bus state or bitrate.
    lely::io::CanController ctrl("can0");
    lely::io::CanChannel chan(poll, exec);
    chan.open(ctrl);

    // Create a CANopen master with node-ID 99. The master is asynchronous, which
    // means every user-defined callback for a CANopen event will be posted as a
    // task on the event loop, instead of being invoked during the event
    // processing by the stack.
    lely::canopen::AsyncMaster master(timer, chan, "master.dcf", "", 99);

    // Create a driver for the slave with node-ID 2.
    MyDriver driver1(exec, master, 1);
    MyDriver driver2(exec, master, 2);
    MyDriver driver3(exec, master, 3);
    MyDriver driver4(exec, master, 4);
    MyDriver driver5(exec, master, 5);
    MyDriver driver6(exec, master, 6);
    MyDriver driver7(exec, master, 7);
    MyDriver driver8(exec, master, 8);

    // Create a signal handler.
    lely::io::SignalSet sigset(poll, exec);
    // Watch for Ctrl+C or process termination.
    sigset.insert(SIGHUP);
    sigset.insert(SIGINT);
    sigset.insert(SIGTERM);

    // Submit a task to be executed when a signal is raised. We don't care which.
    sigset.submit_wait([&](int /*signo*/) {
        // If the signal is raised again, terminate immediately.
        sigset.clear();

        // Tell the master to start the deconfiguration process for all nodes, and
        // submit a task to be executed once that process completes.
        master.AsyncDeconfig().submit(exec, [&]() {
            // Perform a clean shutdown.
            ctx.shutdown();
        });
    });

    // Start the NMT service of the master by pretending to receive a 'reset
    // node' command.
    master.Reset();


    // Run the event loop until no tasks remain (or the I/O context is shut down).
    loop.run();
    std::cout << "testing lely cpp" << std::endl;


    return 0;
}