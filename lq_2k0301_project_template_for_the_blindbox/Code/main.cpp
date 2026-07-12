#include "main.hpp"

BayWatcher_Menu& menu = BayWatcher_Menu::getInstance();
BayWatcher_Motor motor_sys;
BayWatcher_ESC esc_sys;
BayWatcher_Key key_sys;

static volatile bool g_running = true;

static void handle_signal(int)
{
    g_running = false;
}

static void* key_thread_entry(void*)
{
    while (g_running) {
        key_sys.Tick();
        usleep(10000);
    }
    return NULL;
}

static void start_key_thread()
{
    pthread_t tid;
    if (pthread_create(&tid, NULL, key_thread_entry, NULL) == 0) {
        pthread_detach(tid);
    }
}

int main()
{
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    motor_sys.init();
    esc_sys.init();
    key_sys.init();
    menu.init();

    start_key_thread();

    while (g_running) {
        usleep(100000);
    }

    motor_sys.Stop();
    esc_sys.stop();
    TFTSPI_dir_cls(u16BLACK);
    TFTSPI_dir_P8X16Str(0, 0, "System stopped", u16YELLOW, u16BLACK);
    TFTSPI_dir_flush();
    return 0;
}
