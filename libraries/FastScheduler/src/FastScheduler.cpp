#include "FastScheduler.h"
#include <PolledTimeout.h>
#include <atomic>
#ifdef ESP8266
#include <interrupts.h>
#include <coredecls.h>
#else
#include <mutex>
#endif
#include "circular_queue/circular_queue_mp.h"

extern "C"
{
    void esp_loop()
    {
        loop();
        run_scheduled_functions(SCHEDULE_FUNCTION_FROM_LOOP);
    }

#ifdef ESP8266
    void __esp_yield();

    extern "C" void esp_yield()
    {
        __esp_yield();
        run_scheduled_functions(SCHEDULE_FUNCTION_WITHOUT_YIELDELAYCALLS);
    }
#endif
}

typedef std::function<bool(void)> mFuncT;

struct scheduled_fn_t
{
    mFuncT mFunc = nullptr;
    esp8266::polledTimeout::periodicFastUs callNow;
    schedule_e policy = SCHEDULE_FUNCTION_FROM_LOOP;
    std::function<bool()> alarm = nullptr;
    scheduled_fn_t() : callNow(esp8266::polledTimeout::periodicFastUs::alwaysExpired) { }
};

// anonymous namespace provides compilation-unit internal linkage
namespace {
    static circular_queue_mp<scheduled_fn_t> schedule_queue(SCHEDULED_FN_MAX_COUNT);
    static schedule_e activePolicy;
#if defined(ESP32)
    static esp8266::polledTimeout::periodicFastMs yieldSchedulerNow(10); // yield every 10ms
#elif !defined(ESP8266)
    static uint32_t yieldIntvl_us;
#endif
};

bool IRAM_ATTR schedule_recurrent_function_us(std::function<bool(void)>&& fn, uint32_t repeat_us,
    std::function<bool(void)>&& alarm, schedule_e policy)
{
    scheduled_fn_t item;
    item.mFunc = std::move(fn);
    if (repeat_us) item.callNow.reset(repeat_us);
    item.policy = policy;
    item.alarm = std::move(alarm);
    return schedule_queue.push(std::move(item));
}

bool IRAM_ATTR schedule_recurrent_function_us(const std::function<bool(void)>& fn, uint32_t repeat_us,
    const std::function<bool()>& alarm, schedule_e policy)
{
    return schedule_recurrent_function_us(std::function<bool(void)>(fn), repeat_us, std::function<bool()>(alarm), policy);
}

bool IRAM_ATTR schedule_function(std::function<void(void)>&& fn, schedule_e policy)
{
    return schedule_recurrent_function_us([fn]() { fn(); return false; }, 0, nullptr, policy);
}

bool IRAM_ATTR schedule_function(const std::function<void(void)>& fn, schedule_e policy)
{
    return schedule_function(std::function<void(void)>(fn), policy);
}

bool run_function(scheduled_fn_t& func)
{
#if defined(ESP32)
    if (yieldSchedulerNow) yield();
#elif defined(ESP8266)
    optimistic_yield(10000);
#else
    if (micros() - yieldIntvl_us > 10000)
    {
        yield();
        yieldIntvl_us = micros();
    }
#endif
    if (func.policy != SCHEDULE_FUNCTION_WITHOUT_YIELDELAYCALLS && activePolicy != SCHEDULE_FUNCTION_FROM_LOOP) return true;
    bool wakeup = func.alarm && func.alarm();
    bool callNow = func.callNow;
    return !(wakeup || callNow) || func.mFunc();
}

void run_scheduled_functions(schedule_e policy)
{
    // Note to the reader:
    // There is no exposed API to remove a scheduled function:
    // Scheduled functions are removed only from this function, and
    // its purpose is that it is never called from an interrupt
    // (always on cont stack).

    static std::atomic<bool> fence(false);
#ifdef ESP8266
    {
        esp8266::InterruptLock lockAllInterruptsInThisScope;
        if (fence.load()) {
            // prevent any recursive calls from yield()
            return;
        }
        fence.store(true);
    }
#else
    if (fence.exchange(true)) return;
#endif

#if defined(ESP32)
    yieldSchedulerNow.reset(10); // yield every 10ms
#elif !defined(ESP8266)
    yieldIntvl_us = micros();
#endif
    activePolicy = policy;

    // run scheduled function:
    // - when its schedule policy allows it anytime
    // - or if we are called at loop() time
    // and
    // - its time policy allows it
    schedule_queue.for_each_rev_requeue(run_function);
    fence.store(false);
}
