#include <iostream>
#include <vector>
#include <queue>
#include <cassert>
#include <cstdint>

/**
 * Host-native simulation of the Queue Draining logic.
 * Since we don't have FreeRTOS on the host, we mock the behavior.
 */

struct MockSample {
    uint32_t ts_ms;
};

// Logic we want to test (conceptually identical to receiveLatestSample)
bool mockReceiveLatest(std::queue<MockSample>& q, MockSample& out) {
    if (q.empty()) {
        return false;
    }
    
    // Get first
    out = q.front();
    q.pop();
    
    // Drain
    while(!q.empty()) {
        out = q.front();
        q.pop();
    }

    return true;
}

void expect(bool cond, const char *msg) {
    if (cond) {
        std::cout << "PASS: " << msg << "\n";
    } else {
        std::cout << "FAIL: " << msg << "\n";
        exit(1);
    }
}

int main() {
    std::cout << "Starting Queue Logic Test (Host-Simulated)...\n";

    // Test 1: Empty queue
    {
        std::queue<MockSample> q;
        MockSample out = {0};
        expect(mockReceiveLatest(q, out) == false, "Empty queue returns false");
    }

    // Test 2: Single sample
    {
        std::queue<MockSample> q;
        q.push({100});
        MockSample out = {0};
        expect(mockReceiveLatest(q, out) == true, "Single sample returns true");
        expect(out.ts_ms == 100, "Correct sample value");
        expect(q.empty(), "Queue is empty after read");
    }

    // Test 3: Multiple samples (Draining)
    {
        std::queue<MockSample> q;
        q.push({100});
        q.push({200});
        q.push({300});
        MockSample out = {0};
        expect(mockReceiveLatest(q, out) == true, "Multiple samples returns true");
        expect(out.ts_ms == 300, "Correctly picked the LATEST sample (300)");
        expect(q.empty(), "Queue is completely drained");
    }

    std::cout << "✅ All Queue Logic tests passed!\n";
    return 0;
}
