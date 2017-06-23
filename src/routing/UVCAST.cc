//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "UVCAST.h"

Define_Module(UVCAST);

const simsignalwrap_t UVCAST::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);

void UVCAST::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);

    if (stage == 0) {
        traci = Veins::TraCIMobilityAccess().get(getParentModule());

        //TODO: Added for Game Theory Solution
        lastTxPower = registerSignal("lastTxPower");
        meanSNR = registerSignal("meanSNR");

        sentDownMACInCCH = registerSignal("sentDownMACInCCH");
        collisions = registerSignal("collisions");
        messagesTransmittedSCF = registerSignal("messagesTransmittedSCF");
        messagesReceivedSCF = registerSignal("messagesReceivedSCF");
        duplicatedMessages = registerSignal("duplicatedMessages");
        messagesTransmitted = registerSignal("messagesTransmitted");
        messagesReceived = registerSignal("messagesReceived");
        isInROI = registerSignal("isInROI");

        wasInROI = false;
        disseminationStarted = false;

        simulation.getSystemModule()->subscribe("disseminationStartTime", this);

        mac = FindModule<Mac1609_4*>::findSubModule(getParentModule());
        assert(mac);

        //TODO: Added for Game Theory Solution
        curTxPower = mac->par("txPower");
        powerLevel = 3;
        myUtilityValue = 0;

        lastNumCollisions = 0;
        totalCollisions = 0;

        CCHStartTimer = new cMessage("CCH start", CCH_START);
        SCHStartTimer = new cMessage("SCH start", SCH_START);

        uint64_t currenTime = simTime().raw();
        uint64_t switchingTime = SWITCHING_INTERVAL_11P.raw();
        double timeToNextSwitch = (double)(switchingTime - (currenTime % switchingTime)) / simTime().getScale();

        // Control Channel is active
        if ((currenTime / switchingTime) % 2 == 0) {
            scheduleAt(simTime() + timeToNextSwitch + SWITCHING_INTERVAL_11P, CCHStartTimer);
            scheduleAt(simTime() + timeToNextSwitch, SCHStartTimer);
        }
        // Service Channel is active
        else {
            scheduleAt(simTime() + timeToNextSwitch, CCHStartTimer);
            scheduleAt(simTime() + timeToNextSwitch + SWITCHING_INTERVAL_11P, SCHStartTimer);
        }
    }
}

void UVCAST::finish() {
    BaseWaveApplLayer::finish();

    if (wasInROI) {
        emit(isInROI, 1);
    } else {
        emit(isInROI, 0);
    }

    //TODO: Added for Game Theory Solution
    emit(lastTxPower, curTxPower);

    emit(collisions, totalCollisions);

    if (!messagesRcvd.empty()) {
        std::ofstream log;
        std::ostringstream o;

        o << "./results/" << par("log_traffic").longValue() << "-" << par("log_replication").longValue() << "-receiver-" << myId;
        log.open(o.str().c_str());

        for (std::map<int, MessageInfoEntry*>::iterator i = messagesRcvd.begin(); i != messagesRcvd.end(); i++) {
            MessageInfoEntry* videoInfo = i->second;

            log << videoInfo->receptionTime << " " << "id " << videoInfo->messageID << " " << "udp " << videoInfo->messageLength << " " << videoInfo->distanceToOrigin << endl;
        }
        log.close();
    }
    simulation.getSystemModule()->unsubscribe("disseminationStartTime", this);
}

void UVCAST::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {

        case SEND_BEACON_EVT: {
            WaveShortMessage* wsm = prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1);
            BeaconMessage* beaconMsg = new BeaconMessage("beacon");

            beaconMsg->setReceivedMessagesArraySize(messagesRcvd.size());
            unsigned int j = 0;
            for (std::map<int, MessageInfoEntry*>::iterator i = messagesRcvd.begin(); i != messagesRcvd.end(); i++) {
                int messageID = i->first;

                beaconMsg->setReceivedMessages(j, messageID);
                j++;
            }

            wsm->encapsulate(beaconMsg);

            Coord rsuPosition = Coord(par("eventOriginX").doubleValue(), par("eventOriginY").doubleValue(), par("eventOriginZ").doubleValue());
            if (simTime() > par("startDataProductionTime").doubleValue() - 3 &&
                    curPosition.distance(rsuPosition) <= par("dataROI").doubleValue() + 300) {

                sendWSM(wsm);
            } else {
                delete wsm;
            }

            scheduleAt(simTime() + par("beaconInterval").doubleValue(), sendBeaconEvt);
            break;
        }

        case BROADCAST_TIMEOUT: {
            MessageInfoEntry* info = (MessageInfoEntry*) msg->getContextPointer();

            if (isInsideROI(info) && isMessageAlive(info)) {
                WaveShortMessage* wsm = createDataMsg(info);
                sendWSM(wsm);

                emit(messagesTransmitted, 1);

                if (isCCHActive()) {
                    emit(sentDownMACInCCH, 1);
                }
            }

            delete info->broadcastTimer;
            info->broadcastTimer = NULL;

            break;
        }

        case SCF_TIMEOUT: {
            MessageInfoEntry* info = (MessageInfoEntry*) msg->getContextPointer();

            if (isInsideROI(info) && isMessageAlive(info)) {
                WaveShortMessage* wsm = createDataMsg(info);
                sendWSM(wsm);

                emit(messagesTransmittedSCF, 1);
                emit(messagesTransmitted, 1);

                if (isCCHActive()) {
                    emit(sentDownMACInCCH, 1);
                }
            }

            delete info->broadcastTimer;
            info->broadcastTimer = NULL;

            break;
        }

        case ENTRY_TIMEOUT: {
            UVCASTNeighborEntry* entry = (UVCASTNeighborEntry*) msg->getContextPointer();

            int address = entry->senderAddress;

            cancelAndDelete(entry->cleanUpTimer);
            entry->cleanUpTimer = NULL;

            neighbors.erase(address);
            break;
        }

        case BACK_TRAFFIC_ENTRY_TIMEOUT: {
            UVCASTNeighborEntry* entry = (UVCASTNeighborEntry*) msg->getContextPointer();
            int address = entry->senderAddress;
            cancelAndDelete(entry->cleanUpTimer);
            entry->cleanUpTimer = NULL;
            lastRequesters.erase(address);

            break;
        }

        case CCH_START: {
            totalCollisions = totalCollisions + mac->statsTXRXLostPackets - lastNumCollisions;
            scheduleAt(simTime() + SWITCHING_INTERVAL_11P + SWITCHING_INTERVAL_11P, CCHStartTimer);
            break;
        }

        case SCH_START: {
            lastNumCollisions = mac->statsTXRXLostPackets;
            scheduleAt(simTime() + SWITCHING_INTERVAL_11P + SWITCHING_INTERVAL_11P, SCHStartTimer);
            break;
        }

        default: {
            if (msg)
                EV << "UV-CAST - Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
            break;
        }
    }
}

void UVCAST::onBeacon(WaveShortMessage* wsm) {
    scfPolicyMissingPackets(wsm);
    insertEntry(wsm->getSenderAddress(), wsm->getSenderPos());

    Coord rsuPosition = Coord(par("eventOriginX").doubleValue(), par("eventOriginY").doubleValue(), par("eventOriginZ").doubleValue());
    // if back-traffic is enabled, then generate it only three seconds before the main dissemination.
    if (par("generateBackTraffic").boolValue() && simTime() > par("startDataProductionTime").doubleValue() - 3
            && curPosition.distance(rsuPosition) <= par("dataROI").doubleValue() + 300) {
        processBackTraffic(wsm->getSenderAddress());
    }
}

void UVCAST::onData(WaveShortMessage* wsm) {
    //TODO: Added for GAme Theory Solution
    if (par("adaptTxPower").boolValue())
        adjustTxPower(wsm);

    MessageInfoEntry* info = extractMsgInfo(wsm);

    if (!isDuplicateMsg(info->messageID)) {
        // Verify if vehicle should do SCF (border vehicle)
        if (isAssignedSCF(wsm->getSenderAddress(), wsm->getSenderPos())) {
            info->SCF = true;
        } else {
            scheduleRebroadcast(info, wsm->getSenderPos(), BROADCAST_TIMEOUT);
        }

        // Store message info
        messagesRcvd[info->messageID] = info;

        emit(messagesReceived, 1);
    } else {
        // If vehicle is in broadcast suppression, then leave it
        if (info->broadcastTimer && info->broadcastTimer->isScheduled()){
            cancelAndDelete(info->broadcastTimer);
            info->broadcastTimer = NULL;
        }
        emit(duplicatedMessages, 1);
    }
}

bool UVCAST::isAssignedSCF(int senderID, Coord senderPos) {
    double min_angle = M_PI;
    double max_angle = 0.0;

    if (neighbors.empty())
        return true;

    double x = (senderPos.x - curPosition.x);
    double y = (senderPos.y - curPosition.y);

    for (std::map<int, UVCASTNeighborEntry*>::iterator i = neighbors.begin(); i != neighbors.end(); i++) {
        int neighborID = i->first;
        UVCASTNeighborEntry* entry = i->second;

        if (senderID == neighborID) continue;

        double a = (entry->position.x - curPosition.x);
        double b = (entry->position.y - curPosition.y);

        double numerator = (x * a) + (y * b);
        double denominator = sqrt((x*x) + (y*y)) * sqrt((a*a) + (b*b));
        double angle = acos(numerator / denominator);

        if (angle < min_angle)
            min_angle = angle;
        if (angle > max_angle)
            max_angle = angle;
    }

    return (min_angle + max_angle < M_PI);

}

void UVCAST::scheduleRebroadcast(MessageInfoEntry* info, Coord senderPos, UVCASTMessageKinds msgType) {
    double delay, percentageDist;

    percentageDist = std::min(senderPos.distance(curPosition), par("avgCommRange").doubleValue()) / par("avgCommRange").doubleValue();

    // Vehicle is at an intersection
    if (traci->getSpeed() == 0) {
        delay = 0.5 * (1.0 - percentageDist) * par("maxDelay").doubleValue();
    } else {
        delay = 0.5 * (2.0 - percentageDist) * par("maxDelay").doubleValue();
    }

    if (par("desynchronize").boolValue())
        delay = desynchronize(delay);

    info->broadcastTimer = new cMessage("broadcast suppression", msgType);
    info->broadcastTimer->setContextPointer((MessageInfoEntry*) info);

    scheduleAt(simTime() + delay, info->broadcastTimer);
}

void UVCAST::scfPolicyMissingPackets(WaveShortMessage* wsm) {
    BeaconMessage* beaconMsg = dynamic_cast<BeaconMessage*>(wsm->decapsulate());

    for (std::map<int, MessageInfoEntry*>::iterator i = messagesRcvd.begin(); i != messagesRcvd.end(); i++) {
        MessageInfoEntry* info = i->second;
        bool received = false;
        int numMsgsRcvd = beaconMsg->getReceivedMessagesArraySize();

        for (int j = 0; j < numMsgsRcvd; j++) {
            int msgRcvdID = beaconMsg->getReceivedMessages(j);
            if (msgRcvdID == info->messageID) {
                received = true;
                break;
            }
        }

        if (!received && info->SCF && !info->broadcastTimer) {
            scheduleRebroadcast(info, wsm->getSenderPos(), SCF_TIMEOUT);
        }
        received = false;
    }

    delete beaconMsg;
}

double UVCAST::desynchronize(double delay) {
    // Adjust dataRateTimer to send message down to MAC only when the
    // Service Channel is active. If current time is Control Channel, then
    // schedule time to the next Service Channel cycle
    uint64_t currenTime = simTime().raw();
    uint64_t switchingTime = SWITCHING_INTERVAL_11P.raw();
    double timeToNextSwitch = (double)(switchingTime - (currenTime % switchingTime)) / simTime().getScale();
    // Control Channel is active
    if ((currenTime / switchingTime) % 2 == 0) {
        int rounds = floor(delay / 0.05);
        delay = timeToNextSwitch + delay + (rounds * 0.05) + 0.000000000001;

    // Service Channel is active
    } else {
        double delay_tmp = delay - timeToNextSwitch;
        if (delay_tmp > 0) {
            int rounds = ceil(delay_tmp / 0.05);
            delay = delay + (rounds * 0.05);
        }
    }

    return delay;
}

bool UVCAST::isCCHActive() {
    uint64_t currenTime = simTime().raw();
    uint64_t switchingTime = SWITCHING_INTERVAL_11P.raw();

    return ((currenTime / switchingTime) % 2 == 0);
}

UVCAST::MessageInfoEntry* UVCAST::extractMsgInfo(WaveShortMessage* wsm) {
    if (isDuplicateMsg(wsm->getSerial())) {
        return messagesRcvd[wsm->getSerial()];
    }
    DataMessage* dataMsg = dynamic_cast<DataMessage*>(wsm->decapsulate());

    MessageInfoEntry* info = new MessageInfoEntry;

    info->messageID = wsm->getSerial();
    info->messageOriginPosition = dataMsg->getMessageOriginPosition();
    info->messageROI = dataMsg->getMessageROI();
    info->messageOriginTime = dataMsg->getMessageOriginTime();
    info->messageTTL = dataMsg->getMessageTTL();
    info->hops = dataMsg->getHops() + 1;
    info->receptionTime = simTime();
    info->messageLength = wsm->getByteLength();
    info->distanceToOrigin = info->messageOriginPosition.distance(curPosition);
    info->SCF = false;
    info->broadcastTimer = NULL;

    if (dataMsg->getSentSCFState()) {
        emit(messagesReceivedSCF, 1);
    }

    delete dataMsg;

    return info;
}

bool UVCAST::isDuplicateMsg(int messageID) {
    return messagesRcvd.find(messageID) != messagesRcvd.end();
}

bool UVCAST::isInsideROI(MessageInfoEntry* info) {
    return info->messageOriginPosition.distance(curPosition) < info->messageROI;
}

bool UVCAST::isMessageAlive(MessageInfoEntry* info) {
    return simTime() < info->messageOriginTime + info->messageTTL;
}

WaveShortMessage* UVCAST::createDataMsg(MessageInfoEntry* info) {
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, type_SCH, dataPriority, 0, info->messageID);

    //TODO: Added for Game Theory Solution
    PhyControlMessage *controlInfo = new PhyControlMessage();
    controlInfo->setTxPower_mW(curTxPower);
    wsm->setControlInfo(dynamic_cast<cObject *>(controlInfo));

    DataMessage* msg = new DataMessage("data");

    msg->setMessageOriginPosition(info->messageOriginPosition);
    msg->setMessageROI(info->messageROI);
    msg->setMessageOriginTime(info->messageOriginTime);
    msg->setMessageTTL(info->messageTTL);
    msg->setHops(info->hops);

    if (info->SCF) {
        msg->setSentSCFState(true);
    }

    wsm->setByteLength(info->messageLength);

    wsm->encapsulate(msg);

    return wsm;
}

void UVCAST::insertEntry(int senderAddr, Coord senderPos) {
    if (neighbors.find(senderAddr) == neighbors.end()) {
        UVCASTNeighborEntry* entry = new UVCASTNeighborEntry;

        entry->senderAddress = senderAddr;
        entry->position = senderPos;

        entry->cleanUpTimer = new cMessage("entry timeout", ENTRY_TIMEOUT);
        entry->cleanUpTimer->setContextPointer((UVCASTNeighborEntry*) entry);
        scheduleAt(simTime() + par("beaconEntryTTL").doubleValue(), entry->cleanUpTimer);

        neighbors[senderAddr] = entry;
    } else {
        UVCASTNeighborEntry* entry = neighbors[senderAddr];
        entry->position = senderPos;
        cancelEvent(entry->cleanUpTimer);
        scheduleAt(simTime() + par("beaconEntryTTL").doubleValue(), entry->cleanUpTimer);
    }
}

void UVCAST::processBackTraffic(int senderAddr) {
    if (lastRequesters.find(senderAddr) == lastRequesters.end()) {
        // Send 60 packets of 1000 bytes on the Service Channel
        for (int i = 0; i < 60; i++) {
            WaveShortMessage* wsm = prepareWSM("back traffic", dataLengthBits, type_SCH, dataPriority, 0, i);
            wsm->setByteLength(1000);

            sendWSM(wsm);
        }

        UVCASTNeighborEntry* entry = new UVCASTNeighborEntry;

        entry->cleanUpTimer = new cMessage("beacon entry timeout", BACK_TRAFFIC_ENTRY_TIMEOUT);
        entry->cleanUpTimer->setContextPointer((UVCASTNeighborEntry*) entry);
        scheduleAt(simTime() + 3, entry->cleanUpTimer);

        lastRequesters[senderAddr] = entry;
    }
}

void UVCAST::addGPSError() {
    cRNG *rng = getRNG(9);
    double randomNumber, error;

    randomNumber = rng->doubleRand(); // Generate value in [0, 1)
    error = randomNumber * par("maxGPSError").doubleValue();

    if (randomNumber > 0.5) {
        curPosition.x += error;
        curPosition.y += error;
    } else {
        curPosition.x -= error;
        curPosition.y -= error;
    }
}

void UVCAST::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    Enter_Method_Silent();
    BaseWaveApplLayer::receiveSignal(source, signalID, obj, details);

    if (signalID == mobilityStateChangedSignal) {
        Coord rsuPosition = Coord(par("eventOriginX").doubleValue(), par("eventOriginY").doubleValue(), par("eventOriginZ").doubleValue());

        if (disseminationStarted && curPosition.distance(rsuPosition) <= par("dataROI").doubleValue() &&
                ((simTime() >= disseminationStartTime) && (simTime() <= (disseminationStartTime + par("dataTTL").doubleValue())))) {
            wasInROI = true;
        }

        if (par("maxGPSError").doubleValue() > 0) {
            addGPSError();
        }
    }
}

void UVCAST::receiveSignal(cComponent *source, simsignal_t signalID, const SimTime& t) {
    Enter_Method_Silent();

    if (!strcmp(getSignalName(signalID), "disseminationStartTime")) {
        disseminationStarted = true;
        disseminationStartTime = simTime();
        Coord rsuPosition = Coord(par("eventOriginX").doubleValue(), par("eventOriginY").doubleValue(), par("eventOriginZ").doubleValue());

        if (curPosition.distance(rsuPosition) <= par("dataROI").doubleValue())
            wasInROI = true;
    }
}

//TODO: Added for Game Theory Solution
void UVCAST::adjustTxPower(WaveShortMessage* wsm) {
    double rcvTxPower, rcvSNR, utility;

    rcvTxPower = ((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getRecvPower_dBm();
    // Convert dBm to mW
    rcvTxPower = pow(10.0, rcvTxPower / 10.0);
    rcvSNR = ((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getSnr();

    utility = (1 * log(1 + rcvSNR) - (0.009 * curTxPower));

    //if (rcvTxPower >= curTxPower) {
        if (myUtilityValue > utility) {
            myUtilityValue = utility;
            //increaseTxPower();
        } else {
            decreaseTxPower();
        }
    //}

    emit(meanSNR, rcvSNR);
}

//TODO: Added for Game Theory Solution
void UVCAST::decreaseTxPower() {
    const double txPowerVals[] = {0.61, 0.98, 1.6, 2.2};

    if (powerLevel > 0) {
        powerLevel--;
        curTxPower = txPowerVals[powerLevel];
    }
}

//TODO: Added for Game Theory Solution
void UVCAST::increaseTxPower() {
    const double txPowerVals[] = {0.61, 0.98, 1.6, 2.2};

    if (powerLevel < 3) {
        powerLevel++;
        curTxPower = txPowerVals[powerLevel];
    }

}

UVCAST::~UVCAST() {
    for (std::map<int, MessageInfoEntry*>::iterator i = messagesRcvd.begin(); i != messagesRcvd.end(); i++) {
        MessageInfoEntry* info = i->second;

        cancelAndDelete(info->broadcastTimer);
    }

    for (std::map<int, UVCASTNeighborEntry*>::iterator i = neighbors.begin(); i != neighbors.end(); i++) {
        UVCASTNeighborEntry* entry = i->second;

        cancelAndDelete(entry->cleanUpTimer);
    }
    for (std::map<int, UVCASTNeighborEntry*>::iterator i = lastRequesters.begin(); i != lastRequesters.end(); i++) {
        UVCASTNeighborEntry* entry = i->second;
        cancelAndDelete(entry->cleanUpTimer);
    }
    cancelAndDelete(CCHStartTimer);
    cancelAndDelete(SCHStartTimer);
}
