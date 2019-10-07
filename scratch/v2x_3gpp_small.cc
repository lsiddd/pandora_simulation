/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/* * Copyright (c) 2018
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Lucas Pacheco  <lucassidpacheco@gmail.com>
 *         Iago  Medeiros <iagolmedeiros@gmail.com>
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/ipv4-address.h"
#include "ns3/mobility-model.h"
// NetAnim & Evalvid
#include "ns3/netanim-module.h"
#include "ns3/evalvid-client-server-helper.h"
// Pacotes LTE
#include "ns3/point-to-point-helper.h"
#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/lte-module.h"

#include "ns3/string.h"
#include "ns3/double.h"
#include <ns3/boolean.h>
#include <ns3/enum.h>

#include <iomanip>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <typeinfo>

// Used for cell allocation
#include <math.h>
#define PI 3.14159265

#define SIMULATION_TIME_FORMAT(s) Seconds(s)

#define video 1

#if video == 1
#define video_st "sourceTraces/st_highway_cif.st"
#define numberOfFrames 2000
#define numberOfPackets 2106
#define gop 30

#elif video == 2
#define video_st "sourceTraces/st_container_cif_h264_300_20.st"
#define numberOfFrames 300
#define numberOfPackets 614
#define gop 20

#elif video == 3
#define video_st "sourceTraces/st_highway_600_cif"
#define numberOfFrames 600
#define numberOfPackets 716
#define gop 28

#elif video == 4
#define video_st "sourceTraces/st_akiyo_cif_h264_300_18"
#define numberOfFrames 300
#define numberOfPackets 839
#define gop 18

#elif video == 5
#define video_st "sourceTraces/st_football_cif"
#define numberOfFrames 260
#define numberOfPackets 537
#define gop 28

#elif video == 6
#define video_st "sourceTraces/st_mobile_cif_h264_300_18"
#define numberOfFrames 300
#define numberOfPackets 605
#define gop 28

#endif

using namespace ns3;

// do not set this variables, for control only
double TxRate = 0;
unsigned int handNumber = 0;
double qosLastValue = 0;
double qoeLastValue = 0;
int evalvidId = 0;

// simulation options
bool verbose = true;
double simTime = 2; // simlation time
int transmissionStart = 7; // transmission time

// number of user devices
const int node_ue = 100;

// number os cells of each kind
const uint16_t enb_HPN = 2;
const uint16_t low_power = 100;
const uint16_t hot_spot = 0;

// tranmission power for cells
int hpnTxPower = 46;
int lpnTxPower = 23;
int hpTxPower = 15;
const int node_enb = enb_HPN + low_power + hot_spot;

// CBR setting
bool useCbr = false;
uint16_t n_cbr = useCbr ? enb_HPN + low_power : 0;

// topology type
bool randomCellAlloc = true; // random uniform
bool rowTopology = false; // aligned

// control matrix
int cell_ue[enb_HPN + low_power + hot_spot][node_ue];

// exp average parameter
unsigned int exp_mean_window = 3;

// variables for metrics computation
double qoeSum[enb_HPN + low_power + hot_spot];
double qosSum[enb_HPN + low_power + hot_spot];
int qosMetricsIterator[enb_HPN + low_power + hot_spot];
int qoeMetricsIterator[enb_HPN + low_power + hot_spot];

int framePct[numberOfFrames + 1];
std::string frameTypeGlobal[numberOfFrames];
int LastReceivedFrame[node_ue];
bool receivedFrames[node_ue][numberOfPackets];
bool receivedPackets[node_ue][numberOfPackets];
/*----------------------------------------------------------------------*/

NS_LOG_COMPONENT_DEFINE("v2x_3gpp");

/*------------------------- NOTIFICAÇÕES DE HANDOVER ----------------------*/
void NotifyConnectionEstablishedUe(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti)
{
    NS_LOG_DEBUG(Simulator::Now().GetSeconds()
        << " " << context << " UE IMSI " << imsi
        << ": connected to CellId " << cellid << " with RNTI " << rnti);

    std::stringstream temp_cell_dir;
    std::stringstream ueId;
    temp_cell_dir << "./v2x_temp/" << cellid;
    ueId << temp_cell_dir.str() << "/" << rnti;
    if (mkdir(temp_cell_dir.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) {
    }
    std::ofstream outfile(ueId.str().c_str());
    outfile << imsi << std::endl;
    outfile.close();

    cell_ue[cellid - 1][imsi - 1] = rnti;
}

void NotifyHandoverStartUe(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti,
    uint16_t targetCellId)
{
    NS_LOG_DEBUG(Simulator::Now().GetSeconds()
        << " " << context << " UE IMSI " << imsi
        << ": previously connected to CellId " << cellid << " with RNTI "
        << rnti << ", doing handover to CellId " << targetCellId);

    cell_ue[cellid - 1][imsi - 1] = 0;

    std::stringstream ueId;
    ueId << "./v2x_temp/" << cellid << "/" << rnti;
    remove(ueId.str().c_str());

    ++handNumber;
}

void NotifyHandoverEndOkUe(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti)
{
    NS_LOG_DEBUG(Simulator::Now().GetSeconds()
        << " " << context << " UE IMSI " << imsi
        << ": successful handover to CellId " << cellid << " with RNTI "
        << rnti);

    std::stringstream target_cell_dir;
    std::stringstream newUeId;
    target_cell_dir << "./v2x_temp/" << cellid;
    newUeId << target_cell_dir.str() << "/" << rnti;
    if (mkdir(target_cell_dir.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) {
    }
    std::ofstream outfile(newUeId.str().c_str());
    outfile << imsi << std::endl;
    outfile.close();

    cell_ue[cellid - 1][imsi - 1] = rnti;
}

void NotifyConnectionEstablishedEnb(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti)
{
    NS_LOG_DEBUG(Simulator::Now().GetSeconds()
        << " " << context << " eNB CellId " << cellid
        << ": successful connection of UE with IMSI " << imsi << " RNTI "
        << rnti);
}

void NotifyHandoverStartEnb(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti,
    uint16_t targetCellId)
{
    NS_LOG_DEBUG(Simulator::Now().GetSeconds()
        << " " << context << " eNB CellId " << cellid
        << ": start handover of UE with IMSI " << imsi << " RNTI "
        << rnti << " to CellId " << targetCellId);
}

void NotifyHandoverEndOkEnb(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti)
{
    NS_LOG_DEBUG(Simulator::Now().GetSeconds()
        << " " << context << " eNB CellId " << cellid
        << ": completed handover of UE with IMSI " << imsi << " RNTI "
        << rnti);
}

void ArrayPositionAllocator(Ptr<ListPositionAllocator> HpnPosition)
{
    int x, y;
    int distance = 1000;
    std::ofstream outfile("v2x_temp/cellsList", std::ios::out | std::ios::trunc);

    if (randomCellAlloc) {
        for (int i = 0; i < enb_HPN + low_power + hot_spot; ++i) {
            x = rand() % 100 ;
            y = rand() % 2000 ;
            HpnPosition->Add(Vector(x, y, 15));
            outfile << i + 1 << " " << x << " " << y << std::endl;
        }
        outfile.close();
        return;
    }

    else if (rowTopology) {
        int x_start = 700;
        int y_start = 500;
        for (int i = 0; i < enb_HPN + low_power + hot_spot; ++i)
            HpnPosition->Add(Vector(x_start + distance * i, y_start, 25));
        return;
    }

    else {
        int x_start = 1000;
        int y_start = 1000;

        HpnPosition->Add(Vector(x_start, y_start, 25));

        for (double i = 0; i < 2 * PI; i += PI / 3) {
            HpnPosition->Add(Vector(x_start + distance * cos(i), y_start + distance * sin(i), 25));
        }

        for (double i = 0; i < 2 * PI; i += PI / 3) {
            HpnPosition->Add(Vector(x_start + distance * cos(i) + rand() % 100 + 10, y_start + distance * sin(i) + rand() % 100 + 10, 10));
            HpnPosition->Add(Vector(x_start + distance * cos(i) + rand() % 100 + 10, y_start + distance * sin(i) - rand() % 100 + 10, 10));
            HpnPosition->Add(Vector(x_start + distance * cos(i) + rand() % 100 - 10, y_start + distance * sin(i) + rand() % 100 + 10, 10));
            HpnPosition->Add(Vector(x_start + distance * cos(i) + rand() % 100 - 10, y_start + distance * sin(i) - rand() % 100 + 10, 10));
        }
    }
}

void VideoTraceParse(std::string m_videoTraceFileName)
{
    std::ifstream VideoTraceFile(m_videoTraceFileName.c_str(), ios::in);
    if (VideoTraceFile.fail()) {
        NS_FATAL_ERROR(">> EvalvidServer: Error while opening video trace file: "
            << m_videoTraceFileName.c_str());
        return;
    }

    uint32_t frameId;
    std::string frameType;
    uint32_t frameSize;
    uint16_t numOfUdpPackets;
    double sendTime;
    uint32_t packet = 0;

    while (VideoTraceFile >> frameId >> frameType >> frameSize >> numOfUdpPackets >> sendTime) {
        framePct[frameId - 1] = packet + 1;
        frameTypeGlobal[frameId - 1] = frameType;
        packet += numOfUdpPackets;
    }
}

std::string exec(const char* cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
        throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }
    return result;
}

/*------------------------- CRIAÇÃO ARQUIVO COM QOS'S ----------------------*/
void WriteMetrics()
{
    NS_LOG_DEBUG(Simulator::Now().GetSeconds() << " Segundos...");
    NS_LOG_DEBUG("Realizados " << handNumber << " Handover");
    for (int i = 0; i < node_enb; ++i) {
        for (int u = 0; u < node_ue; ++u)
            if (cell_ue[i][u]) {
                std::stringstream rdTrace;
                rdTrace << "v2x_temp/rd_a01_" << u;
                std::ifstream rdFile(rdTrace.str());
                if (!rdFile) {
                    NS_LOG_DEBUG("NO FILE TO BE READ" << '\n');
                    return;
                }
                double rdTime;
                std::string id;
                int rdPacketId;
                string a;
                int b;

                int lastPacket = 0;
                int counter;
                int npackets;

                while (rdFile >> rdTime >> id >> rdPacketId >> a >> b) {
                    receivedPackets[u][rdPacketId - 1] = true;
                    lastPacket = rdPacketId;
                }

                /*----------------QOS METRIC CALCULATION----------------*/
                int nReceived = 0;
                for (int j = lastPacket - 60; j <= lastPacket; ++j) {
                    if (receivedPackets[u][j])
                        ++nReceived;
                }
                if (lastPacket >= 60) {
                    /*NS_LOG_INFO("Taxa de recebimento, node " << u << " :"
                                                             << ((float)nReceived - 1) / 60);*/
                    //NS_LOG_INFO ("Recebidos " << nReceived << " pacotes.");
                    stringstream qosFilename;
                    double valorAtualQos = 0;
                    qosFilename << "./v2x_temp/qosTorre" << i + 1;
                    ifstream qosInFile(qosFilename.str());
                    while (qosInFile >> valorAtualQos) {
                    }
                    ofstream qosOutFile(qosFilename.str(),
                        std::ofstream::out | std::ofstream::trunc);

                    //CÁLCULO DA MÉDIA EXPONENCIAL
                    qosOutFile << 2 * (((float)nReceived - 1) / 60 - valorAtualQos) / (exp_mean_window + 1) + valorAtualQos;
                    NS_LOG_DEBUG("NODE " << u << " QOS ESTIMADO " << 2 * (((float)nReceived - 1) / 60 - valorAtualQos) / (exp_mean_window + 1) + valorAtualQos);

                    //CALCULO DE QOS POR MÉDIA SIMPLES
                    //qosSum[i] += ((float)nReceived - 1) / 60;
                    //qosMetricsIterator[i]++;
                    //qosOutFile << qosSum[i] / qosMetricsIterator[i];
                    //NS_LOG_DEBUG("NODE " << u << " QOS ESTIMADO " << qosSum[i] / qosMetricsIterator[i]);
                }

                /**-------------------------------------------------------*/
                /*-----------------QOE METRIC CALCULATION-----------------*/
                /*--------------------------------------------------------*/

                framePct[numberOfFrames] = numberOfPackets + 1;
                for (int j = 0; j < numberOfFrames; ++j) {
                    npackets = framePct[j + 1] - framePct[j];
                    counter = 0;
                    if (framePct[j] > lastPacket) {
                        continue;
                    }

                    for (int k = framePct[j]; k < framePct[j + 1]; ++k)
                        if (receivedPackets[u][k - 1]) {
                            ++counter;
                        }

                    if (npackets == counter) {
                        receivedFrames[u][j] = true;
                        LastReceivedFrame[u] = j + 1;
                    }
                }

                int lastGop = 0;
                int IReceived = 0;
                int ITotal = 0;
                double ILoss = 0;

                int PReceived = 0;
                int PTotal = 0;
                double PLoss = 0;

                int BReceived = 0;
                int BTotal = 0;
                double BLoss = 0;

                for (int j = 0; j < LastReceivedFrame[u] / 20; j++)
                    ++lastGop;

                if (lastGop != 0) {
                    for (int j = 20 * (lastGop - 1); j < lastGop * 20 - 1; ++j) {
                        if (frameTypeGlobal[j].find("I") != std::string::npos || frameTypeGlobal[j].find("H") != std::string::npos)
                            ++ITotal;
                        else if (frameTypeGlobal[j].find("P") != std::string::npos)
                            ++PTotal;
                        else if (frameTypeGlobal[j].find("B") != std::string::npos)
                            ++BTotal;

                        if (receivedFrames[u][j]) {
                            if (frameTypeGlobal[j].find("I") != std::string::npos || frameTypeGlobal[j].find("H") != std::string::npos)
                                ++IReceived;
                            else if (frameTypeGlobal[j].find("P") != std::string::npos)
                                ++PReceived;
                            else if (frameTypeGlobal[j].find("B") != std::string::npos)
                                ++BReceived;
                        }
                    }
                    ILoss = ITotal != 0 ? ((double)ITotal - (double)IReceived) * 100 / (double)ITotal : 0;
                    PLoss = PTotal != 0 ? ((double)PTotal - (double)PReceived) * 100 / (double)PTotal : 0;
                    BLoss = BTotal != 0 ? ((double)BTotal - (double)BReceived) * 100 / (double)BTotal : 0;

                    std::stringstream cmd;
                    std::stringstream qoeFileName;
                    std::string qoeResult;

                    qoeFileName << "./v2x_temp/qoeTorre" << i + 1;
                    double valorAtualQoe = 0;
                    ifstream qoeInFile(qoeFileName.str());
                    while (qoeInFile >> valorAtualQoe) {
                    }
                    // ESTE ARQUIVO CONTÉM O VALOR DA ÚLTIMA MÉDIA CALCULADA
                    ofstream qoeOutFile(qoeFileName.str(),
                        std::ofstream::out | std::ofstream::trunc);

                    cmd << "python2.7 ia.py " << ILoss << " " << PLoss << " " << BLoss
                        << " " << gop << " 2>/dev/null";

                    //CÁLCULO DA MÉDIA EXPONENCIAL E ESCRITA NO ARQUIVO
                    qoeOutFile << 2 * (stod(exec(cmd.str().c_str())) - valorAtualQoe) / (exp_mean_window + 1) + valorAtualQoe;
                    NS_LOG_INFO("NODE " << u << " QOE ESTIMADO " << 2 * (stod(exec(cmd.str().c_str())) - valorAtualQoe) / (exp_mean_window + 1) + valorAtualQoe);
                    qoeOutFile.close();

                    //CÁLCULO POR MÉDIA SIMPLES
                    // qoeSum[i] += stod(exec(cmd.str().c_str()));
                    // qoeMetricsIterator[i]++;
                    // qoeOutFile << qoeSum[i] / qoeMetricsIterator[i];

                    //NS_LOG_INFO("NODE " << u << " QOE ESTIMADO " << qoeSum[i] / qoeMetricsIterator[i]);
                }
            }
    }
    NS_LOG_INFO("\n\n\n\n");
    return;
}

void requestStream(Ptr<Node> remoteHost, NodeContainer ueNodes, Ipv4Address remoteHostAddr, double simTime, double start)
{
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i) {
        string video_trans = video_st;
        int startTime = rand() % 10 + transmissionStart;
        NS_LOG_UNCOND("Node " << i << " requesting video at " << startTime << "\n");
        std::stringstream sdTrace;
        std::stringstream rdTrace;
        std::stringstream rdWindow;
        sdTrace << "v2x_temp/sd_a01_" << evalvidId;
        rdTrace << "v2x_temp/rd_a01_" << evalvidId;
        evalvidId++;

        double stop = simTime;
        uint16_t m_port = 2000 * evalvidId + 2000; // Para alcançar o nó ZERO quando i = 0

        // Servidor de vídeo
        EvalvidServerHelper server(m_port);
        server.SetAttribute("SenderTraceFilename", StringValue(video_trans));
        server.SetAttribute("SenderDumpFilename", StringValue(sdTrace.str()));
        server.SetAttribute("PacketPayload", UintegerValue(512));
        ApplicationContainer apps = server.Install(remoteHost);
        apps.Start(Seconds(1));
        apps.Stop(Seconds(stop));

        // Clientes do vídeo
        EvalvidClientHelper client(remoteHostAddr, m_port);
        client.SetAttribute("ReceiverDumpFilename", StringValue(rdTrace.str()));
        apps = client.Install(ueNodes.Get(i));
        apps.Start(Seconds(startTime));
        apps.Stop(Seconds(stop));

        Ptr<Ipv4> ipv4 = ueNodes.Get(i)->GetObject<Ipv4>();
    }
}

int main(int argc, char* argv[])
{

    if (mkdir("./v2x_temp", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) {
    }
    system("exec rm -rf v2x_temp/*");

    for (int u = 0; u < node_ue; ++u) {
        for (int i = 0; i < numberOfPackets; ++i)
            receivedPackets[u][i] = false;

        for (int i = 0; i < numberOfFrames; ++i)
            receivedFrames[u][i] = false;
    }

    int seedValue = 1;
    std::string handoverAlg = "hove";
    CommandLine cmm;

    cmm.AddValue("seedValue", "random seed value.", seedValue);
    cmm.AddValue("handoverAlg", "Handover algorith in use.", handoverAlg);
    cmm.AddValue("verbose", "lte stats verbose.", verbose);
    cmm.Parse(argc, argv);
    RngSeedManager::SetSeed(seedValue + 10000); //valor de seed para geração de números aleatórios
    srand(seedValue);

    double interPacketInterval = 1;
    VideoTraceParse(video_st);

    LogComponentEnable("v2x_3gpp", LOG_LEVEL_DEBUG);
    LogComponentEnable("v2x_3gpp", LOG_LEVEL_INFO);
    LogComponentEnable("EvalvidClient", LOG_LEVEL_INFO);
    if (verbose) {
        LogComponentEnable("LteEnbRrc", LOG_LEVEL_ALL);
        LogComponentEnable("LteUeRrc", LOG_LEVEL_ALL);
        // LogComponentEnable("LteUePhy", LOG_LEVEL_ALL);
        // LogComponentEnable("LteEnbPhy", LOG_LEVEL_ALL);
        LogComponentEnable("HoveHandoverAlgorithm", LOG_LEVEL_INFO);
        LogComponentEnable("HoveHandoverAlgorithm", LOG_LEVEL_DEBUG);
    }

    uint16_t node_remote = 1; // HOST_REMOTO
    for (double t = transmissionStart; t < simTime; t += 1)
        Simulator::Schedule(Seconds(t), &WriteMetrics);

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults();

    /*------------------------- LTE MODULE ----------------------*/
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    // EPC helper
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    lteHelper->SetEpcHelper(epcHelper);

    //*********** CONFIGURAÇÃO LTE ***************//

    // bandwidth in resource blocks
    Config::SetDefault("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(25));
    Config::SetDefault("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(25));

    // ctrl and data channel models
    Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (true));
    if (handoverAlg == "hove")
        Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (false));

    // transmission mode (SISO [0], MIMO [1])
    Config::SetDefault("ns3::LteEnbRrc::DefaultTransmissionMode",
        UintegerValue(1));
    // path loss model
    lteHelper->SetAttribute("PathlossModel",
        StringValue("ns3::NakagamiPropagationLossModel"));
    // UL and DL frequencies
    lteHelper->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(100));
    lteHelper->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(18100));
    lteHelper->SetSchedulerType("ns3::PssFfMacScheduler");
    // the maximum number of UE selected by TD scheduler
    lteHelper->SetSchedulerAttribute( "nMux", UintegerValue(1));
    // PF scheduler type in PSS
    lteHelper->SetSchedulerAttribute("PssFdSchedulerType", StringValue("CoItA"));

    // RRC model
    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(true));

    // Antenna parameters
    lteHelper->SetEnbAntennaModelType("ns3::CosineAntennaModel");
    lteHelper->SetEnbAntennaModelAttribute("Orientation", DoubleValue(0));
    lteHelper->SetEnbAntennaModelAttribute("Beamwidth", DoubleValue(60));
    lteHelper->SetEnbAntennaModelAttribute("MaxGain", DoubleValue(0.0));


    // Handover algorithm selection
    if (handoverAlg == "hove") {
        lteHelper->SetHandoverAlgorithmType("ns3::HoveHandoverAlgorithm");
        lteHelper->SetHandoverAlgorithmAttribute("NumberOfUEs", UintegerValue(node_ue));
    }

    else if (handoverAlg == "skip") {
        lteHelper->SetHandoverAlgorithmType("ns3::SkippingHandoverAlgorithm");
        lteHelper->SetHandoverAlgorithmAttribute("NumberOfUEs", UintegerValue(node_ue));
    }

    else if (handoverAlg == "ser") {
        lteHelper->SetHandoverAlgorithmType("ns3::SerHandoverAlgorithm");
        lteHelper->SetHandoverAlgorithmAttribute("NumberOfUEs", UintegerValue(node_ue));
    }

    else if (handoverAlg == "qoe")
        lteHelper->SetHandoverAlgorithmType("ns3::QoeHandoverAlgorithm");

    else if (handoverAlg == "noop")
        lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm");

    else if (handoverAlg == "a3") {
        lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
        lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(3.0));
        lteHelper->SetHandoverAlgorithmAttribute("TimeToTrigger",
            TimeValue(MilliSeconds(256)));
    }

    else if (handoverAlg == "a2a4") {
        lteHelper->SetHandoverAlgorithmType("ns3::A2A4RsrqHandoverAlgorithm");
        lteHelper->SetHandoverAlgorithmAttribute("ServingCellThreshold",
            UintegerValue(30));
        lteHelper->SetHandoverAlgorithmAttribute("NeighbourCellOffset",
            UintegerValue(2));
    }

    else {
        NS_LOG_ERROR("Invalid handover algorithm option.");
    }


    //-------------Criação do RemoteHost
    // Cria um simples RemoteHost
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(node_remote);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    // Pilha de Internet
    InternetStackHelper internet;
    internet.Install(remoteHost);

    // Cria link Internet
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1400));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    // Determina endereço ip para o Link
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("10.1.0.0", "255.255.0.0");
    Ipv4InterfaceContainer internetIpIfaces;
    internetIpIfaces = ipv4h.Assign(internetDevices);

    // interface 0 é localhost e interface 1 é dispositivo p2p
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
        Ipv4Mask("255.0.0.0"), 1);

    NodeContainer nodes_ue_nc;
    nodes_ue_nc.Create(node_ue);

    NodeContainer cbr_nodes;
    cbr_nodes.Create(n_cbr);

    NodeContainer enbNodes;
    enbNodes.Create(enb_HPN + low_power + hot_spot);

    internet.Install(nodes_ue_nc);
    internet.Install(cbr_nodes);

    /*-----------------POSIÇÃO DAS TORRES----------------------------------*/
    Ptr<ListPositionAllocator> HpnPosition = CreateObject<ListPositionAllocator>();
    ArrayPositionAllocator(HpnPosition);

    MobilityHelper remoteHostMobility;
    remoteHostMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    remoteHostMobility.Install(remoteHost);
    remoteHostMobility.Install(pgw);

    /*-----------------MONILIDADE DAS TORRES --------------*/

    MobilityHelper mobilityEnb;
    mobilityEnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityEnb.SetPositionAllocator(HpnPosition);
    mobilityEnb.Install(enbNodes);

    MobilityHelper mobilityCbr;
    mobilityEnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityEnb.SetPositionAllocator(HpnPosition);
    mobilityEnb.Install(cbr_nodes);

    Ns2MobilityHelper ue_mobil = Ns2MobilityHelper("mobil/SanFrancisco.tcl");
    MobilityHelper ueMobility;
    MobilityHelper enbMobility;

    ue_mobil.Install(nodes_ue_nc.Begin(), nodes_ue_nc.End());

    //-------------Instala LTE Devices para cada grupo de nós
    NetDeviceContainer enbLteDevs;
    enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer pedLteDevs;
    NetDeviceContainer carLteDevs;
    carLteDevs = lteHelper->InstallUeDevice(nodes_ue_nc);
    NetDeviceContainer cbrLteDevs;
    cbrLteDevs = lteHelper->InstallUeDevice(cbr_nodes);

    /*----------------------------------------------------------------------*/

    Ipv4InterfaceContainer pedIpIface;
    pedIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(pedLteDevs));
    Ipv4InterfaceContainer carIpIface;
    carIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(carLteDevs));
    Ipv4InterfaceContainer cbrIpFace;
    cbrIpFace = epcHelper->AssignUeIpv4Address(NetDeviceContainer(cbrLteDevs));

    for (uint32_t u = 0; u < nodes_ue_nc.GetN(); ++u) {
        Ptr<Node> ueNode = nodes_ue_nc.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(),
            1);
        // Simulator::Schedule(Seconds(0.0), &showPosition, ueNode, 1.0);
    }

    /*-------------------------CONFIGURAÇÃO DE CBR-------------------------*/
    uint16_t cbrPort = 3000;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;

    //--------------------DEFINIR GATEWAY---------------------
    for (uint32_t u = 0; u < cbr_nodes.GetN(); ++u) {
        Ptr<Node> ueNode = cbr_nodes.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(),
            1);

        Ptr<Ipv4> ipv4 = cbr_nodes.Get(u)->GetObject<Ipv4>();
        Ipv4InterfaceAddress iaddr = ipv4->GetAddress(1, 0);
        Ipv4Address addri = iaddr.GetLocal();
        PacketSinkHelper packetSinkHelper(
            "ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), cbrPort));
        serverApps.Add(packetSinkHelper.Install(cbr_nodes.Get(u)));
        serverApps.Start(Seconds(20));

        int load = 1024;
        UdpClientHelper client(addri, cbrPort);
        client.SetAttribute("Interval",
            TimeValue(MilliSeconds(interPacketInterval)));
        client.SetAttribute("MaxPackets", UintegerValue(100000000));
        client.SetAttribute("PacketSize", UintegerValue(load));
        NS_LOG_INFO("Célula " << u + 1 << " com cbr de " << load << " bytes" << std::endl);

        clientApps.Add(client.Install(remoteHost));

        clientApps.Start(Seconds(20));
    }

    /*-----------------POTENCIA DE TRASMISSAO-----------------*/
    Ptr<LteEnbPhy> enb0Phy;

    // todo: set different bandwidth for individual cells
    for (int i = 0; (unsigned)i < enbLteDevs.GetN(); i++) {
        enb0Phy = enbLteDevs.Get(i)->GetObject<LteEnbNetDevice>()->GetPhy();
        if (i < enb_HPN) {
            enb0Phy->SetTxPower(hpnTxPower);
        }
        else if (i < enb_HPN + low_power) {
            enb0Phy->SetTxPower(lpnTxPower);
        }
        else if (i < enb_HPN + low_power + hot_spot) {
            enb0Phy->SetTxPower(lpnTxPower);
        }
    }

    //-------------Anexa as UEs na eNodeB

    lteHelper->Attach(carLteDevs);
    lteHelper->AttachToClosestEnb(cbrLteDevs, enbLteDevs);
    lteHelper->AddX2Interface(enbNodes);

    NS_LOG_INFO("Create Applications.");

    // Início Transmissão de Vídeo
    //Rodar aplicação EvalVid
    requestStream(remoteHost, nodes_ue_nc, remoteHostAddr, simTime, transmissionStart);

    /*----------------NETANIM-------------------------------*/
    AnimationInterface anim("v2x_temp/LTEnormal_v2x.xml");
    // Cor e Descrição para eNb
    for (uint32_t i = 0; i < enbNodes.GetN(); ++i) {
        anim.UpdateNodeDescription(enbNodes.Get(i), "eNb");
        anim.UpdateNodeColor(enbNodes.Get(i), 0, 255, 0);
    }
    for (uint32_t i = 0; i < nodes_ue_nc.GetN(); ++i) {
        anim.UpdateNodeDescription(nodes_ue_nc.Get(i), "UE Carro");
        anim.UpdateNodeColor(nodes_ue_nc.Get(i), 255, 0, 0);
    }
    for (uint32_t i = 0; i < cbr_nodes.GetN(); ++i) {
        anim.UpdateNodeDescription(cbr_nodes.Get(i), "CBR");
        anim.UpdateNodeColor(cbr_nodes.Get(i), 0, 255, 0);
    }
    anim.UpdateNodeDescription(remoteHost, "RH");
    anim.UpdateNodeColor(remoteHost, 0, 255, 255);

    Simulator::Stop(SIMULATION_TIME_FORMAT(simTime));

    /*--------------NOTIFICAÇÕES DE HANDOVER E SINAL-------------------------*/
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
        MakeCallback(&NotifyConnectionEstablishedUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
        MakeCallback(&NotifyHandoverStartUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
        MakeCallback(&NotifyHandoverEndOkUe));

    //lteHelper->EnableTraces();

    Simulator::Run();
    Simulator::Destroy();

    return EXIT_SUCCESS;
}
