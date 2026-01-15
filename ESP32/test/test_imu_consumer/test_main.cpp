#include <unity.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "imu_drivers/IIMUDriver.h"
#include "imu_consumer_helpers.h"

// Mock de la structure IMUSample pour le test
using abbot::IMUSample;

void test_queue_draining_single_sample() {
    QueueHandle_t testQueue = xQueueCreate(5, sizeof(IMUSample));
    IMUSample s1;
    s1.ts_ms = 100;
    xQueueSend(testQueue, &s1, 0);

    IMUSample result;
    bool ok = abbot::imu_consumer::receiveLatestSample(testQueue, result, 0);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT32(100, result.ts_ms);
    TEST_ASSERT_EQUAL_UINT32(0, uxQueueMessagesWaiting(testQueue));

    vQueueDelete(testQueue);
}

void test_queue_draining_multiple_samples() {
    QueueHandle_t testQueue = xQueueCreate(10, sizeof(IMUSample));
    
    // On simule l'accumulation de 3 samples
    IMUSample s1, s2, s3;
    s1.ts_ms = 100;
    s2.ts_ms = 110;
    s3.ts_ms = 120; // Le plus récent
    
    xQueueSend(testQueue, &s1, 0);
    xQueueSend(testQueue, &s2, 0);
    xQueueSend(testQueue, &s3, 0);

    IMUSample result;
    // On appelle notre fonction de draining
    bool ok = abbot::imu_consumer::receiveLatestSample(testQueue, result, 0);

    TEST_ASSERT_TRUE(ok);
    // On vérifie qu'on a bien récupéré SEULEMENT le plus récent
    TEST_ASSERT_EQUAL_UINT32(120, result.ts_ms);
    // On vérifie que la queue est maintenant vide
    TEST_ASSERT_EQUAL_UINT32(0, uxQueueMessagesWaiting(testQueue));

    vQueueDelete(testQueue);
}

void test_queue_draining_empty_queue() {
    QueueHandle_t testQueue = xQueueCreate(5, sizeof(IMUSample));
    IMUSample result;
    
    // Test sur queue vide avec timeout 0
    bool ok = abbot::imu_consumer::receiveLatestSample(testQueue, result, 0);
    
    TEST_ASSERT_FALSE(ok);
    
    vQueueDelete(testQueue);
}

void setup() {
    delay(2000); // Wait for serial
    UNITY_BEGIN();
    RUN_TEST(test_queue_draining_single_sample);
    RUN_TEST(test_queue_draining_multiple_samples);
    RUN_TEST(test_queue_draining_empty_queue);
    UNITY_END();
}

void loop() {
    // Nothing to do
}
