#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define STACK_SIZE      1024
#define THREAD_PRIORITY 5

K_MUTEX_DEFINE(counter_mutex);

/* Volatile does NOT make the compound read-modify-write operation atomic. */
static volatile int shared_counter;

static void increment_100(const char *name)
{
    for (int i = 0; i < 100; i++) {

        /*
         * Keep BOTH mutex calls enabled for the correct version.
         * Comment BOTH calls to reproduce the race condition.
         */
        ///k_mutex_lock(&counter_mutex, K_FOREVER);

        int local = shared_counter;

        /*
         * Deliberately bad practice for this exercise only:
         * sleeping here makes the race condition easy to observe
         * when the mutex calls are commented out.
         */
        k_sleep(K_MSEC(1));

        shared_counter = local + 1;

        ///k_mutex_unlock(&counter_mutex);
    }

    printk("Thread %s finished\n", name);
}

static void thread_a(void *a, void *b, void *c)
{
    (void)a;
    (void)b;
    (void)c;
    increment_100("A");
}

static void thread_b(void *a, void *b, void *c)
{
    (void)a;
    (void)b;
    (void)c;
    increment_100("B");
}

/* Small start delay so main() can print the exercise header first. */
K_THREAD_DEFINE(thread_a_id, STACK_SIZE, thread_a,
                NULL, NULL, NULL,
                THREAD_PRIORITY, 0, 100);

K_THREAD_DEFINE(thread_b_id, STACK_SIZE, thread_b,
                NULL, NULL, NULL,
                THREAD_PRIORITY, 0, 100);

int main(void)
{
    printk("Embedded Platforms and Communications for IoT\n");
    printk("        ETSIST - UPM - MUIoT 2026-2027       \n\n");
    printk("RTOS mutex solution: two threads increment the same counter.\n");
    printk("Expected result WITH mutex: 200\n\n");

    /* Wait until both worker threads have terminated. */
    k_thread_join(thread_a_id, K_FOREVER);
    k_thread_join(thread_b_id, K_FOREVER);

    printk("\nFinal counter = %d (expected 200 with mutex)\n", shared_counter);
    printk("Now comment ONLY k_mutex_lock() and k_mutex_unlock(), rebuild and compare.\n");

    return 0;
}
