#include "Buzzer.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <thread>

namespace {

std::atomic_bool running{true};

void handle_signal(int)
{
    running = false;
}

} // namespace

BayWatcher_Buzzer buzzer_sys;

int main()
{
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    std::printf("Buzzer test started (GPIO %d).\n", BUZZER_PIN);
    std::printf("The buzzer will beep for 500 ms every second. Press Ctrl+C to stop.\n");

    buzzer_sys.init();

    while (running) {
        buzzer_sys.beep(500);

        // beep() is non-blocking, so Tick() must be called periodically.
        for (int elapsed_ms = 0; elapsed_ms < 1000 && running; elapsed_ms += 10) {
            buzzer_sys.Tick();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    buzzer_sys.off();
    std::printf("\nBuzzer test stopped.\n");
    return 0;
}
