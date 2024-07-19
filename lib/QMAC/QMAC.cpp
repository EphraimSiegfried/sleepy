#include <QMAC.h>

bool QMACClass::begin(int64_t sleepingDuration, int64_t activeDuration,
                      byte localAddress) {
    // Setting up the attributes:
    this->localAddress =
        localAddress == BCADDR
            ? random(254)
            : localAddress;  // assign random address if address not specified
    this->msgCount = 1;
    this->sleepingDuration = sleepingDuration;
    this->activeDuration = activeDuration;
    signalDetected = false;

    // Creating and starting the activity switching timer, then synchronize it
    // with the other nodes:
    esp_timer_create_args_t timer_args = {.callback = &QMACClass::timerCallback,
                                          .arg = this,
                                          .name = "duty_cycle_timer"};
    esp_timer_create(&timer_args, &this->timer_handle);
    esp_timer_start_once(timer_handle, sleepingDuration * 1000);
    // synchronize();
    LoRa.onCadDone(QMAC.cadCallback);
    LoRa.onReceive(QMAC.receiveCallback);

    LOG("Local Address: " + String(this->localAddress));
    return true;
}
void QMACClass::timerCallback(void* arg) {
    QMACClass* self = static_cast<QMACClass*>(arg);
    if (self->active) {
        LoRa.sleep();
        esp_timer_start_once(self->timer_handle, self->sleepingDuration * 1000);
    } else {
        LoRa.channelActivityDetection();
        esp_timer_start_once(self->timer_handle, self->activeDuration * 1000);
    }
    self->active = !self->active;
}

ICACHE_RAM_ATTR void QMACClass::cadCallback(boolean signal) {
    if (signal) {
        LoRa.receive();
        QMAC.signalDetected = true;
    } else {
        QMAC.signalDetected = false;
        LoRa.channelActivityDetection();
    }
}

ICACHE_RAM_ATTR void QMACClass::receiveCallback(int packetSize) {
    Packet p;
    QMAC.receive(&p, packetSize);
    if (p.isSyncPacket()) {
        Packet syncResponse = {
            .destination = p.destination,
            .source = QMAC.localAddress,
            .msgCount = 0,
            .nextActiveTime = QMAC.nextActiveTime(),
            .payloadLength = 0,
        };
        QMAC.sendQueue.addFirst(p);
    } else if (p.isAck()) {
        for (size_t i = 0; i < QMAC.unackedQueue.getSize(); i++) {
            if (QMAC.unackedQueue[i].msgCount == p.msgCount) {
                QMAC.unackedQueue.remove(i);
                break;
            }
        }
    } else {
        bool isAlreadyReceived = false;
        for (size_t i = 0; i < QMAC.receptionQueue.getSize(); i++) {
            if (QMAC.receptionQueue[i].msgCount == p.msgCount) {
                isAlreadyReceived = true;
                break;
            }
        }
        if (!isAlreadyReceived) {
            QMAC.receptionQueue.add(p);
        }
        // Don't send ACKs if destination is broadcast address:
        if (p.destination != BCADDR) {
            Packet ackPacket = {
                .destination = p.source,
                .source = QMAC.localAddress,
                .msgCount = p.msgCount,
                .payloadLength = 0,
            };
            QMAC.sendQueue.addFirst(p);
        }
    }
}

void QMACClass::run() {
    while (this->active && !QMAC.signalDetected && !sendQueue.isEmpty()) {
        Packet nextPacket = QMAC.sendQueue[0];
        QMAC.sendPacket(nextPacket);
        QMAC.sendQueue.removeFirst();
        if (nextPacket.destination != BCADDR) {
            QMAC.unackedQueue.add(nextPacket);
        }
        LoRa.channelActivityDetection();
    }
}

int QMACClass::amountAvailable() { return receptionQueue.getSize(); }

bool QMACClass::push(String payload, byte destination) {
    // Creating a new packets with all required fields:
    Packet p;
    p.destination = destination;
    p.source = localAddress;
    p.msgCount = this->msgCount++;
    p.payloadLength = payload.length();
    payload.getBytes(p.payload, sizeof(p.payload));
    sendQueue.add(p);
    return true;
}

bool QMACClass::sendPacket(Packet p) {
    if (!LoRa.beginPacket()) {
        LOG("LoRa beginPacket failed");
        return false;
    }
    CRC16 crc;
    // Sends all fields of the packet in order:
    LoRa.write(p.destination);  // add destination address
    crc.add(p.destination);
    LoRa.write(p.source);  // add sender address
    crc.add(p.source);
    LoRa.write(p.msgCount);  // add message ID
    crc.add(p.msgCount);
    if (p.isSyncPacket()) {
        // convert nextActiveTime to byte array
        byte b[2] = {p.nextActiveTime & 0xff, p.nextActiveTime >> 8};
        LoRa.write(b, 2);
        crc.add(b, 2);
    } else {
        LoRa.write(p.payloadLength);
        crc.add(p.payloadLength);
        LoRa.write(p.payload, p.payloadLength);
        crc.add(p.payload, p.payloadLength);
    }
    uint16_t checksum = crc.calc();
    byte c[2] = {checksum & 0xff, checksum >> 8};
    LoRa.write(c, 2);
    if (!LoRa.endPacket()) {
        LOG("LoRa endPacket failed");
        return false;
    }
    return true;
}

bool QMACClass::receive(Packet* p, int packetSize) {
    if (!packetSize) return false;
    // Parses the received data as a packet:
    CRC16 crc;
    p->destination = LoRa.read();
    crc.add(p->destination);
    p->source = LoRa.read();
    crc.add(p->source);
    p->msgCount = LoRa.read();
    crc.add(p->msgCount);
    if (p->isSyncPacket()) {
        byte t[2];
        LoRa.readBytes(t, 2);
        p->nextActiveTime = *((int*)t);
        crc.add(t, 2);
    } else {
        p->payloadLength = LoRa.read();
        crc.add(p->payloadLength);
        LoRa.readBytes(p->payload, p->payloadLength);
        crc.add(p->payload, p->payloadLength);
    }
    byte checksum[2];
    LoRa.readBytes(checksum, 2);
    // return true if CRC check is successfull
    // TODO: should the checksum be added to the Packet struct? should checking
    // be done outside of the receive function?
    return crc.calc() == *((uint16_t*)checksum);
}

uint16_t QMACClass::nextActiveTime() {
    int64_t nextTimeout = esp_timer_get_next_alarm() / 1000 - millis();
    // If active, next active period will happen after the next sleeping period.
    // If sleeping, next active period will happen directly after the end of the
    // current period.
    return active ? nextTimeout + this->sleepingDuration : nextTimeout;
}

// void QMACClass::synchronize() {
//     LOG("Start synchronization");
//     List<uint16_t> receivedTimestamps;
//     List<uint64_t> transmissionDelays;
//     List<uint64_t> receptionTimestamps;

//     // Sending sync packets and waiting until a response is received:
//     while (1) {
//         long transmissionStartTime = millis();
//         sendSyncPacket(BCADDR);

//         // listen for sync responses for some time
//         long listeningStartTime = millis();
//         while (millis() - listeningStartTime < this->activeDuration) {
//             Packet p = {};
//             if (!receive(&p)) continue;
//             if (p.isSyncPacket()) {
//                 transmissionDelays.add(millis() - transmissionStartTime);
//                 receivedTimestamps.add(p.nextActiveTime);
//                 receptionTimestamps.add(millis());
//             }
//         }

//         // stop if response received
//         if (!receivedTimestamps.isEmpty()) break;

//         // go to sleep if no responses received
//         LoRa.sleep();
//         delay(this->activeDuration + random(-1 / 2 * this->activeDuration,
//                                             1 / 2 * this->activeDuration));
//     }

//     int numResponses = receivedTimestamps.getSize();
//     uint64_t averageNextActiveTime = 0;
//     for (size_t i = 0; i < numResponses; i++) {
//         LOG("RECEIVED TIMESTAMP " + String(receivedTimestamps[i]));
//         averageNextActiveTime += receivedTimestamps[i] -
//                                  0.5 * transmissionDelays[i] -
//                                  (millis() - receptionTimestamps[i]);
//     }
//     averageNextActiveTime += nextActiveTime();
//     averageNextActiveTime = averageNextActiveTime / (numResponses + 1);
//     LOG("Own Schedule " + String(nextActiveTime()));
//     LOG("Average next time active " + String(averageNextActiveTime));

//     updateTimer(averageNextActiveTime);
// }

void QMACClass::updateTimer(uint64_t timeUntilActive) {
    esp_timer_stop(this->timer_handle);
    this->active = false;  // Node will be forced to sleep until reaching
                           // the new next active time
    esp_timer_start_once(this->timer_handle, timeUntilActive * 1000);
}

QMACClass QMAC;
