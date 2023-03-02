/*
 *  Copyright (c) 2017, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <openthread/config.h>

#include <memory>

#include "common/array.hpp"
#include "common/code_utils.hpp"
#include "common/instance.hpp"

#include "ncp/ncp_base.hpp"
#include "openthread/link_raw.h"

#include "test_platform.h"
#include "test_util.hpp"

using namespace ot;
using namespace ot::Ncp;

enum
{
    kTestBufferSize = 800
};

enum
{
    kTestMacScanChannelMask = 0x0001
};

OT_TOOL_PACKED_BEGIN
struct RadioMessage
{
    uint8_t mChannel;
    uint8_t mPsdu[OT_RADIO_FRAME_MAX_SIZE];
} OT_TOOL_PACKED_END;

static struct RadioMessage sDefaultMessage;
static otRadioFrame sDefaultFrame;

static otRadioFrame *sTxFrame;

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance * aInstance) { return sTxFrame; }

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *, int8_t *) { return OT_ERROR_NONE; }

otError otPlatRadioGetCoexMetrics(otInstance *, otRadioCoexMetrics *) { return OT_ERROR_NONE; }

otError otPlatRadioGetTransmitPower(otInstance *, int8_t *) { return OT_ERROR_NONE; }

bool otPlatRadioIsCoexEnabled(otInstance *) { return true; }

otError otPlatRadioSetCoexEnabled(otInstance *, bool) { return OT_ERROR_NOT_IMPLEMENTED; }


#if OPENTHREAD_POSIX_CONFIG_MAX_POWER_TABLE_ENABLE
otError otPlatRadioSetChannelTargetPower(otInstance *aInstance, uint8_t aChannel, int16_t aTargetPower) { return OT_ERROR_NONE; }

otError otPlatRadioAddCalibratedPower(otInstance    *aInstance,
                                      uint8_t        aChannel,
                                      int16_t        aActualPower,
                                      const uint8_t *aRawPowerSetting,
                                      uint16_t       aRawPowerSettingLength) 
{ return OT_ERROR_NONE; }

otError otPlatRadioClearCalibratedPowers(otInstance *aInstance) { return OT_ERROR_NONE; }

#endif

class TestNcp : public NcpBase
{
    public:
        TestNcp(ot::Instance *aInstance) : mLastHeader(0), mLastStatus(0),
        NcpBase(aInstance)
        {
            memset(mMsgBuffer, 0, kTestBufferSize);
            mTxFrameBuffer.SetFrameAddedCallback(HandleFrameAddedToNcpBuffer, this);
            mTxFrameBuffer.SetFrameRemovedCallback(nullptr, this);
        };

        static void HandleFrameAddedToNcpBuffer(void                    *aContext,
                                                Spinel::Buffer::FrameTag aTag,
                                                Spinel::Buffer::Priority aPriority,
                                                Spinel::Buffer          *aBuffer)
        {
            OT_UNUSED_VARIABLE(aTag);
            OT_UNUSED_VARIABLE(aPriority);

            static_cast<TestNcp *>(aContext)->HandleFrameAddedToNcpBuffer(aBuffer);
        }

        void HandleFrameAddedToNcpBuffer(Spinel::Buffer *aBuffer)
        {
            static const size_t display_size = 64;

            memset(mMsgBuffer, 0, kTestBufferSize);
            SuccessOrQuit(aBuffer->OutFrameBegin());
            aBuffer->OutFrameRead(kTestBufferSize, mMsgBuffer);
            SuccessOrQuit(aBuffer->OutFrameRemove());

            //DumpBuffer("Received Buffer", mMsgBuffer, display_size);

            updateSpinelStatus();
        }

        void Receive(uint8_t *aBuffer, size_t bufferSize)
        {
            HandleReceive(aBuffer, bufferSize);
        }

        void processTransmit()
        {
            LinkRawTransmitDone(sTxFrame, nullptr, OT_ERROR_NONE);
            //printf("Transmit Finished\n");
            processPendingCommands(); // Pending commands tasklet is posted by Transmit Done callback but not handled
        };

        void processEnergyScan()
        {
            LinkRawEnergyScanDone(Radio::kInvalidRssi);
            //printf("Energy Scan Finished\n");
            //processPendingCommands(); Pending commands are handled in Energy Scan Done callback
        }

        void processPendingCommands()
        {
            #if OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE && \
                (OPENTHREAD_RADIO || OPENTHREAD_CONFIG_LINK_RAW_ENABLE)
            HandlePendingCommands();
            #endif
        }

        void updateSpinelStatus()
        {
            Spinel::Decoder decoder;

            uint8_t header;
            unsigned int command;
            spinel_prop_key_t propKey;
            spinel_status_t status;

            decoder.Init(mMsgBuffer, kTestBufferSize);

            SuccessOrQuit(decoder.ReadUint8(mLastHeader));
            SuccessOrQuit(decoder.ReadUintPacked(command));
            SuccessOrQuit(decoder.ReadUintPacked(propKey));
            SuccessOrQuit(decoder.ReadUintPacked(mLastStatus));
        }
        uint32_t getSpinelStatus()
        {
            return mLastStatus;
        }

        uint8_t getLastIid()
        {
            return SPINEL_HEADER_IID_MASK & mLastHeader;
            //return SPINEL_HEADER_GET_IID(mLastHeader);
        }

        uint8_t getLastTid()
        {
            return SPINEL_HEADER_GET_TID(mLastHeader);
        }

        bool gotResponse(uint8_t aIid, uint8_t aTid)
        {
            return ((aIid == getLastIid()) &&
                    (aTid == getLastTid()));
        }

        size_t getPendingQueueSize()
        { 
        #if OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE && \
            (OPENTHREAD_RADIO || OPENTHREAD_CONFIG_LINK_RAW_ENABLE)
            return GetPendingCommandQueueSize();
        #else
            return 0;
        #endif
        };

        size_t getMaxPendingQueueSize()
        {
        #if OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE && \
            (OPENTHREAD_RADIO || OPENTHREAD_CONFIG_LINK_RAW_ENABLE)
            return kPendingCommandQueueSize; 
        #else
            return 0;
        #endif
        };

    private:
        uint8_t mLastHeader;
        uint32_t mLastStatus;
        uint8_t mMsgBuffer[kTestBufferSize];
};

class TestHost
{
    public:
        TestHost(std::shared_ptr<TestNcp> aNcp, uint8_t aIid) :
        mNcp(aNcp),
        mIid(aIid),
        mTid(0),
        mBuffer(mBuf, kTestBufferSize),
        mEncoder(mBuffer),
        mOffset(0)
        {
            memset(mBuf, 0, kTestBufferSize);
        };

        void createLinkEnableFrame(bool isEnabled)
        {        
            startFrame(mEncoder, SPINEL_CMD_PROP_VALUE_SET, SPINEL_PROP_PHY_ENABLED);
            SuccessOrQuit(mEncoder.WriteBool(isEnabled));
            endFrame(mEncoder, "Enable Frame");
        }

        void createTransmitFrame()
        {
            startFrame(mEncoder, SPINEL_CMD_PROP_VALUE_SET, SPINEL_PROP_STREAM_RAW);

            SuccessOrQuit(mEncoder.WriteDataWithLen(sDefaultFrame.mPsdu, sDefaultFrame.mLength));
            SuccessOrQuit(mEncoder.WriteUint8(sDefaultFrame.mChannel));
            SuccessOrQuit(mEncoder.WriteUint8(sDefaultFrame.mInfo.mTxInfo.mMaxCsmaBackoffs));
            SuccessOrQuit(mEncoder.WriteUint8(sDefaultFrame.mInfo.mTxInfo.mMaxFrameRetries));
            SuccessOrQuit(mEncoder.WriteBool(sDefaultFrame.mInfo.mTxInfo.mCsmaCaEnabled));
            SuccessOrQuit(mEncoder.WriteBool(sDefaultFrame.mInfo.mTxInfo.mIsHeaderUpdated));
            SuccessOrQuit(mEncoder.WriteBool(sDefaultFrame.mInfo.mTxInfo.mIsARetx));
            SuccessOrQuit(mEncoder.WriteBool(sDefaultFrame.mInfo.mTxInfo.mIsSecurityProcessed));
            SuccessOrQuit(mEncoder.WriteUint32(sDefaultFrame.mInfo.mTxInfo.mTxDelay));
            SuccessOrQuit(mEncoder.WriteUint32(sDefaultFrame.mInfo.mTxInfo.mTxDelayBaseTime));

            endFrame(mEncoder, "Transmit Frame");
        }

        void createScanChannelMaskFrame(uint32_t aMask)
        {
            startFrame(mEncoder, SPINEL_CMD_PROP_VALUE_SET, SPINEL_PROP_MAC_SCAN_MASK);
            SuccessOrQuit(mEncoder.WriteUint8(aMask));
            endFrame(mEncoder, "Channel Mask Frame");
        }

        void createMacScanFrame()
        {
            uint8_t state = SPINEL_SCAN_STATE_ENERGY;

            startFrame(mEncoder, SPINEL_CMD_PROP_VALUE_SET, SPINEL_PROP_MAC_SCAN_STATE);
            SuccessOrQuit(mEncoder.WriteUint8(state));
            endFrame(mEncoder, "Scan State Frame");
        }

        void createReadStatusFrame()
        {
            startFrame(mEncoder, SPINEL_CMD_PROP_VALUE_GET, SPINEL_PROP_LAST_STATUS);
            endFrame(mEncoder, "Read Status Frame");
        }

        void enableRawLink()
        {
            static const bool isLinkEnabled = true;
            createLinkEnableFrame(isLinkEnabled);
            sendToRcp();
        }

        void disableRawLink()
        {
            static const bool isLinkEnabled = false;
            createLinkEnableFrame(isLinkEnabled);
            sendToRcp();
        }

        spinel_status_t startTransmit() 
        {
            static const bool getResponse = true;
            createTransmitFrame();
            sendToRcp(getResponse);
            return (spinel_status_t)(mNcp->getSpinelStatus());
        };

        void setScanChannelMask(uint32_t aMask)
        {
            createScanChannelMaskFrame(aMask);
            sendToRcp();
        }

        uint32_t startEnergyScan()
        {
            static const bool getResponse = true;
            createMacScanFrame();
            sendToRcp(getResponse);
            return mNcp->getSpinelStatus();
        }

        void getCommandStatus() 
        {
            createReadStatusFrame();
            sendToRcp();
        }

        void finishTransmit()
        {
            // Resetting instance submac state to sleep by resetting link
            disableRawLink();
            enableRawLink();

            mNcp->processTransmit();
        };

    private:

        void startFrame(Spinel::Encoder &aEncoder, unsigned int aCommand, spinel_prop_key_t aKey)
        {
            uint8_t spinelHeader = SPINEL_HEADER_FLAG | mIid | mTid;
            
            SuccessOrQuit(aEncoder.BeginFrame(Spinel::Buffer::kPriorityLow));
            SuccessOrQuit(aEncoder.WriteUint8(spinelHeader));
            SuccessOrQuit(aEncoder.WriteUintPacked(aCommand));
            SuccessOrQuit(aEncoder.WriteUintPacked(aKey));
        }

        void endFrame(Spinel::Encoder &aEncoder, const char *aTextMessage)
        {
            static const uint16_t display_length = 64; 
            SuccessOrQuit(aEncoder.EndFrame());
            //DumpBuffer(aTextMessage, mBuf, display_length);
        }

        void sendToRcp(bool needsResponse = false)
        {
            static const uint8_t data_offset = 2;
            bool requestResponse = false;

            size_t frame_len = mBuffer.OutFrameGetLength();

            mOffset += data_offset;
            
            mNcp->Receive(mBuf + mOffset, frame_len);

            requestResponse = needsResponse && !mNcp->gotResponse(mIid, mTid);

            mTid = SPINEL_GET_NEXT_TID(mTid);
            SuccessOrQuit(mBuffer.OutFrameRemove());

            mOffset += frame_len;
            mOffset %= kTestBufferSize;

            if (requestResponse)
            {
                getCommandStatus();
            }
        }

        std::shared_ptr<TestNcp> mNcp;
        uint8_t mIid;
        uint8_t mTid;
        uint8_t mBuf[kTestBufferSize];
        Spinel::Buffer mBuffer;
        Spinel::Encoder mEncoder;
        size_t mOffset;
};

void TestNcpBaseTransmitWithLinkRawDisabled()
{
    ot::Instance *instance = testInitInstance();
    VerifyOrQuit(instance != nullptr);

    std::shared_ptr<TestNcp> ncp = std::make_shared<TestNcp>(instance);
    TestHost host(ncp, SPINEL_HEADER_IID_0);
   
    host.disableRawLink();
    VerifyOrQuit(host.startTransmit() == SPINEL_STATUS_INVALID_STATE);

    testFreeInstance(instance);
}

void TestNcpBaseTransmitWithLinkRawEnabled()
{
    ot::Instance *instance = testInitInstance();
    VerifyOrQuit(instance != nullptr);

    std::shared_ptr<TestNcp> ncp = std::make_shared<TestNcp>(instance);
    TestHost host(ncp, SPINEL_HEADER_IID_0);

    host.enableRawLink();

    VerifyOrQuit(host.startTransmit() == SPINEL_STATUS_OK);
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    host.finishTransmit();

    testFreeInstance(instance);
}

void TestNcpBaseTransmitWithNoBuffers()
{
    sTxFrame = nullptr;

    ot::Instance *instance = testInitInstance();
    VerifyOrQuit(instance != nullptr);

    std::shared_ptr<TestNcp> ncp = std::make_shared<TestNcp>(instance);
    TestHost host(ncp, SPINEL_HEADER_IID_0);

    host.enableRawLink();

    VerifyOrQuit(host.startTransmit() == SPINEL_STATUS_NOMEM);
    
    testFreeInstance(instance);

    sTxFrame = &sDefaultFrame;
}

void TestNcpBaseTransmitWhileLinkIsBusy()
{
    ot::Instance *instance = testInitInstance();
    VerifyOrQuit(instance != nullptr);

    std::shared_ptr<TestNcp> ncp = std::make_shared<TestNcp>(instance);
    TestHost host(ncp, SPINEL_HEADER_IID_0);

    host.enableRawLink();
    
    VerifyOrQuit(host.startTransmit() == SPINEL_STATUS_OK);
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    VerifyOrQuit(host.startTransmit() == SPINEL_STATUS_OK);
    VerifyOrQuit(ncp->getPendingQueueSize() == 1);

    VerifyOrQuit(host.startTransmit() == SPINEL_STATUS_OK);
    VerifyOrQuit(ncp->getPendingQueueSize() == 2);

    host.finishTransmit();
    VerifyOrQuit(ncp->getPendingQueueSize() == 1);

    host.finishTransmit();
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    host.finishTransmit();
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    testFreeInstance(instance);
}

void TestNcpBaseExceedPendingCommandQueueSize()
{
    ot::Instance *instance = testInitInstance();
    VerifyOrQuit(instance != nullptr);

    std::shared_ptr<TestNcp> ncp = std::make_shared<TestNcp>(instance);
    TestHost host(ncp, SPINEL_HEADER_IID_0);

    host.enableRawLink();
    
    for (size_t i = 0; i <= ncp->getMaxPendingQueueSize(); i++)
    {
        VerifyOrQuit(host.startTransmit() == SPINEL_STATUS_OK);
        VerifyOrQuit(ncp->getPendingQueueSize() == i);
    }
    //printf("Queue Size: %zu\n", ncp->getPendingQueueSize());

    VerifyOrQuit(host.startTransmit() == SPINEL_STATUS_NOMEM);
    VerifyOrQuit(ncp->getPendingQueueSize() == ncp->getMaxPendingQueueSize());

    //printf("Queue Size: %zu\n", ncp->getPendingQueueSize());

    for (size_t i = ncp->getMaxPendingQueueSize(); i > 0; i--)
    {
        VerifyOrQuit(ncp->getPendingQueueSize() == i);
        host.finishTransmit();
    }

    VerifyOrQuit(ncp->getPendingQueueSize() == 0);
    host.finishTransmit();
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    testFreeInstance(instance);
}

void TestNcpBaseEnergyScanWithLinkRawDisabled()
{
    ot::Instance *instance = testInitInstance();
    VerifyOrQuit(instance != nullptr);

    std::shared_ptr<TestNcp> ncp = std::make_shared<TestNcp>(instance);
    TestHost host(ncp, SPINEL_HEADER_IID_0);

    host.disableRawLink();
    VerifyOrQuit((spinel_status_t)host.startEnergyScan() == SPINEL_STATUS_OK);
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    //Put test condition here. Maybe that the queue won't fill for invalid energy scan without error

    VerifyOrQuit((spinel_status_t)host.startEnergyScan() == SPINEL_STATUS_OK);
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    testFreeInstance(instance);
}

void TestNcpBaseEnergyScanWithLinkRawEnabled()
{
    ot::Instance *instance = testInitInstance();
    VerifyOrQuit(instance != nullptr);

    std::shared_ptr<TestNcp> ncp = std::make_shared<TestNcp>(instance);
    TestHost host(ncp, SPINEL_HEADER_IID_0);

    host.enableRawLink();
    VerifyOrQuit((spinel_status_t)host.startEnergyScan() == SPINEL_STATUS_INVALID_ARGUMENT);

    testFreeInstance(instance);
}

void TestNcpBaseEnergyScanWithLinkRawEnabledAndMaskSet()
{
    ot::Instance *instance = testInitInstance();
    VerifyOrQuit(instance != nullptr);

    std::shared_ptr<TestNcp> ncp = std::make_shared<TestNcp>(instance);
    TestHost host(ncp, SPINEL_HEADER_IID_0);
   
    host.enableRawLink();
    host.setScanChannelMask(kTestMacScanChannelMask);

    VerifyOrQuit((spinel_scan_state_t)host.startEnergyScan() == SPINEL_SCAN_STATE_ENERGY);

    ncp->processEnergyScan();

    testFreeInstance(instance);
}

void TestNcpBaseEnergyScanWhileLinkIsBusy()
{
    ot::Instance *instance = testInitInstance();
    VerifyOrQuit(instance != nullptr);

    std::shared_ptr<TestNcp> ncp = std::make_shared<TestNcp>(instance);
    TestHost host(ncp, SPINEL_HEADER_IID_0);
    
    otError ret = OT_ERROR_NONE;
    spinel_status_t spinel_ret = SPINEL_STATUS_OK;

    host.enableRawLink();  
    host.setScanChannelMask(kTestMacScanChannelMask);

    VerifyOrQuit((spinel_scan_state_t)host.startEnergyScan() == SPINEL_SCAN_STATE_ENERGY);
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    VerifyOrQuit((spinel_status_t)host.startEnergyScan() == SPINEL_STATUS_INVALID_STATE);
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    //These calls do not update the queue - Omit from test?
    VerifyOrQuit((spinel_status_t)host.startEnergyScan() == SPINEL_STATUS_INVALID_STATE);
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    ncp->processEnergyScan();
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    VerifyOrQuit((spinel_scan_state_t)host.startEnergyScan() == SPINEL_SCAN_STATE_ENERGY);
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    ncp->processEnergyScan();
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    testFreeInstance(instance);
}

void TestNcpBaseEnergyScanWhileTransmitting()
{
    ot::Instance *instance = testInitInstance();
    VerifyOrQuit(instance != nullptr);

    std::shared_ptr<TestNcp> ncp = std::make_shared<TestNcp>(instance);
    TestHost host(ncp, SPINEL_HEADER_IID_0);

    host.enableRawLink();
    
    VerifyOrQuit(host.startTransmit() == SPINEL_STATUS_OK);
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    host.setScanChannelMask(kTestMacScanChannelMask);

    VerifyOrQuit((spinel_scan_state_t)host.startEnergyScan() == SPINEL_SCAN_STATE_IDLE);
    VerifyOrQuit(ncp->getPendingQueueSize() == 1);

    VerifyOrQuit((spinel_scan_state_t)host.startEnergyScan() == SPINEL_SCAN_STATE_IDLE);
    VerifyOrQuit(ncp->getPendingQueueSize() == 2);

    host.finishTransmit();
    VerifyOrQuit(ncp->getPendingQueueSize() == 1);

    ncp->processEnergyScan();
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    ncp->processEnergyScan();
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    testFreeInstance(instance);
}

void TestNcpBaseTransmitWhileScanning()
{
    ot::Instance *instance = testInitInstance();
    VerifyOrQuit(instance != nullptr);

    std::shared_ptr<TestNcp> ncp = std::make_shared<TestNcp>(instance);
    TestHost host(ncp, SPINEL_HEADER_IID_0);

    host.enableRawLink();
    host.setScanChannelMask(kTestMacScanChannelMask);

    VerifyOrQuit((spinel_scan_state_t)host.startEnergyScan() == SPINEL_SCAN_STATE_ENERGY);
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);  

    VerifyOrQuit(host.startTransmit() == SPINEL_STATUS_OK);
    VerifyOrQuit(ncp->getPendingQueueSize() == 1);  

    ncp->processEnergyScan();
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    host.finishTransmit();
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    testFreeInstance(instance);
}

void TestNcpBaseMultiHostTransmit()
{
    ot::Instance *instance = testInitInstance();
    VerifyOrQuit(instance != nullptr);

    std::shared_ptr<TestNcp> ncp = std::make_shared<TestNcp>(instance);
    TestHost host0(ncp, SPINEL_HEADER_IID_1), host1(ncp, SPINEL_HEADER_IID_2);

    host0.enableRawLink();
    host1.enableRawLink();

    VerifyOrQuit(host0.startTransmit() == SPINEL_STATUS_OK);
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);  

    host0.finishTransmit();
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);  

    VerifyOrQuit(host0.startTransmit() == SPINEL_STATUS_OK);
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    host1.enableRawLink();

    VerifyOrQuit(host1.startTransmit() == SPINEL_STATUS_OK);
    VerifyOrQuit(ncp->getPendingQueueSize() == 1);

    host0.finishTransmit();
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    host1.finishTransmit();
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);
}

void TestNcpBaseMultiHostEnergyScan()
{
    ot::Instance *instance = testInitInstance();
    VerifyOrQuit(instance != nullptr);

    std::shared_ptr<TestNcp> ncp = std::make_shared<TestNcp>(instance);
    TestHost host0(ncp, SPINEL_HEADER_IID_1), host1(ncp, SPINEL_HEADER_IID_2);

    host0.enableRawLink();
    host0.setScanChannelMask(kTestMacScanChannelMask);

    host1.enableRawLink();
    host1.setScanChannelMask(kTestMacScanChannelMask);

    VerifyOrQuit((spinel_scan_state_t)host0.startEnergyScan() == SPINEL_SCAN_STATE_ENERGY);
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);  

    ncp->processEnergyScan();
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);  

    VerifyOrQuit((spinel_scan_state_t)host0.startEnergyScan() == SPINEL_SCAN_STATE_ENERGY);
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    VerifyOrQuit((spinel_status_t)host1.startEnergyScan() == SPINEL_STATUS_INVALID_STATE);
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    ncp->processEnergyScan();
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    VerifyOrQuit((spinel_scan_state_t)host1.startEnergyScan() == SPINEL_SCAN_STATE_ENERGY);
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);

    ncp->processEnergyScan();
    VerifyOrQuit(ncp->getPendingQueueSize() == 0);
}

int main(void)
{
    sDefaultFrame.mPsdu = sDefaultMessage.mPsdu;
    sTxFrame = &sDefaultFrame;

#if OPENTHREAD_CONFIG_MULTIPAN_RCP_ENABLE && \
    (OPENTHREAD_RADIO || OPENTHREAD_CONFIG_LINK_RAW_ENABLE)

    TestNcpBaseTransmitWithLinkRawDisabled();
    TestNcpBaseTransmitWithLinkRawEnabled();
    TestNcpBaseTransmitWithNoBuffers();
    TestNcpBaseTransmitWhileLinkIsBusy();
    TestNcpBaseExceedPendingCommandQueueSize();
    TestNcpBaseEnergyScanWithLinkRawDisabled();
    TestNcpBaseEnergyScanWithLinkRawEnabled();
    TestNcpBaseEnergyScanWithLinkRawEnabledAndMaskSet();
    TestNcpBaseEnergyScanWhileLinkIsBusy();
    TestNcpBaseEnergyScanWhileTransmitting();
    TestNcpBaseTransmitWhileScanning();
    TestNcpBaseMultiHostTransmit();
    TestNcpBaseMultiHostEnergyScan();

    printf("\nAll tests passed\n");
#else
    printf("MULTIPAN_RCP feature and RADIO/LINK_RAW option are not enabled\n");
#endif

    return 0;
}