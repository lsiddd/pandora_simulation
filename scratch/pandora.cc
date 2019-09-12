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
 // Pacotes LTE
#include "ns3/point-to-point-helper.h"

#include "ns3/lte-helper.h"

#include "ns3/epc-helper.h"

#include "ns3/lte-module.h"

#include "ns3/string.h"

#include "ns3/double.h"

#include <ns3/boolean.h>

#include <ns3/enum.h>

#include "ns3/flow-monitor-helper.h"

#include "ns3/ipv4-flow-classifier.h"

#include <fstream>

#include <iostream>

#include <sys/stat.h>

#define SIMULATION_TIME_FORMAT(s) Seconds(s)

using namespace ns3;

double TxRate = 0;
bool useCbr = false;
bool verbose = false;

const int node_ue = 20;

const uint16_t enb_HPN = 9;
const uint16_t low_power = 0;
const uint16_t hot_spot = 0;

const int node_enb = enb_HPN + low_power + hot_spot;

uint16_t n_cbr = useCbr ? enb_HPN + low_power : 0;

int hpnTxPower = 46;
int lpnTxPower = 23;
int hpTxPower = 15;

double simTime = 10;

unsigned int handNumber = 0;

bool randomCellAlloc = true;
bool rowTopology = false;

//coeficiente da média exponencial
unsigned int exp_mean_window = 3;
double qosLastValue = 0;
double qoeLastValue = 0;

NS_LOG_COMPONENT_DEFINE("v2x_3gpp");

/*------------------------- NOTIFICAÇÕES DE HANDOVER ----------------------*/
void NotifyConnectionEstablishedUe(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti) {
    NS_LOG_DEBUG(Simulator::Now().GetSeconds() <<
        " " << context << " UE IMSI " << imsi <<
        ": connected to CellId " << cellid << " with RNTI " << rnti);

    std::stringstream temp_cell_dir;
    std::stringstream ueId;
    temp_cell_dir << "./v2x_temp/" << cellid;
    ueId << temp_cell_dir.str() << "/" << rnti;
    if (mkdir(temp_cell_dir.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) {}
    std::ofstream outfile(ueId.str().c_str());
    outfile << imsi << std::endl;
    outfile.close();
}

void NotifyHandoverStartUe(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti,
    uint16_t targetCellId) {
    NS_LOG_DEBUG(Simulator::Now().GetSeconds() <<
        " " << context << " UE IMSI " << imsi <<
        ": previously connected to CellId " << cellid << " with RNTI " <<
        rnti << ", doing handover to CellId " << targetCellId);

    std::stringstream ueId;
    ueId << "./v2x_temp/" << cellid << "/" << rnti;
    remove(ueId.str().c_str());

    ++handNumber;
}

void NotifyHandoverEndOkUe(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti) {
    NS_LOG_DEBUG(Simulator::Now().GetSeconds() <<
        " " << context << " UE IMSI " << imsi <<
        ": successful handover to CellId " << cellid << " with RNTI " <<
        rnti);

    std::stringstream target_cell_dir;
    std::stringstream newUeId;
    target_cell_dir << "./v2x_temp/" << cellid;
    newUeId << target_cell_dir.str() << "/" << rnti;
    if (mkdir(target_cell_dir.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) {}
    std::ofstream outfile(newUeId.str().c_str());
    outfile << imsi << std::endl;
    outfile.close();
}

void NotifyConnectionEstablishedEnb(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti) {
    NS_LOG_DEBUG(Simulator::Now().GetSeconds() <<
        " " << context << " eNB CellId " << cellid <<
        ": successful connection of UE with IMSI " << imsi << " RNTI " <<
        rnti);
}

void NotifyHandoverStartEnb(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti,
    uint16_t targetCellId) {
    NS_LOG_DEBUG(Simulator::Now().GetSeconds() <<
        " " << context << " eNB CellId " << cellid <<
        ": start handover of UE with IMSI " << imsi << " RNTI " <<
        rnti << " to CellId " << targetCellId);
}

void NotifyHandoverEndOkEnb(std::string context,
    uint64_t imsi,
    uint16_t cellid,
    uint16_t rnti) {
    NS_LOG_DEBUG(Simulator::Now().GetSeconds() <<
        " " << context << " eNB CellId " << cellid <<
        ": completed handover of UE with IMSI " << imsi << " RNTI " <<
        rnti);
}

void ArrayPositionAllocator(Ptr < ListPositionAllocator > HpnPosition) {
    std::ifstream pos_file("CellsDataset");
    int cellId;
    double x_coord, y_coord;

    while (pos_file >> cellId >> x_coord >> y_coord) {
        NS_LOG_INFO("Adding cell " << cellId <<
            " to coordinates x:" << x_coord << " y:" << y_coord);
        HpnPosition -> Add(Vector(x_coord, y_coord, 30));
    }
}

void requestService() {}
void migrateService() {}

int main(int argc, char * argv[]) {

    // tmp file that exists while this simulation is running
    int seedValue = 1;
    CommandLine cmm;

    cmm.AddValue("seedValue", "random seed value.", seedValue);
    cmm.AddValue("verbose", "lte stats verbose.", verbose);
    cmm.Parse(argc, argv);
    RngSeedManager::SetSeed(seedValue + 10000); //valor de seed para geração de números aleatórios
    srand(seedValue);

    LogComponentEnable("v2x_3gpp", LOG_LEVEL_DEBUG);
    LogComponentEnable("v2x_3gpp", LOG_LEVEL_INFO);
    if (verbose) {
        LogComponentEnable("LteEnbRrc", LOG_LEVEL_ALL);
        LogComponentEnable("LteUeRrc", LOG_LEVEL_ALL);
        //LogComponentEnable("LteUePhy", LOG_LEVEL_ALL);
        //LogComponentEnable("LteEnbPhy", LOG_LEVEL_ALL);
    }

    //-------------Parâmetros da simulação
    uint16_t node_remote = 1; // HOST_REMOTO
    /*----------------------------------------------------------------------*/

    // Bandwidth of Dl and Ul in Resource Blocks
    Config::SetDefault("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(25));
    Config::SetDefault("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(25));

    // Modo de transmissão (SISO [0], MIMO [1])
    Config::SetDefault("ns3::LteEnbRrc::DefaultTransmissionMode",
        UintegerValue(1));

    Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(320));

    /*------------------------- MÓDULOS LTE ----------------------*/
    Ptr < LteHelper > lteHelper = CreateObject < LteHelper > ();
    Ptr < EpcHelper > epcHelper;
    epcHelper = CreateObject < NoBackhaulEpcHelper > ();

    Ptr < Node > pgw = epcHelper -> GetPgwNode();

    lteHelper -> SetEpcHelper(epcHelper);

    // LTE configuration
    lteHelper -> SetSchedulerType("ns3::PssFfMacScheduler");
    lteHelper -> SetSchedulerAttribute("nMux", UintegerValue(1)); // the maximum number of UE selected by TD scheduler
    lteHelper -> SetSchedulerAttribute("PssFdSchedulerType", StringValue("CoItA")); // PF scheduler type in PSS
    lteHelper -> SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
    lteHelper -> EnableTraces();

    // Propagation Parameters
    lteHelper -> SetEnbDeviceAttribute("DlEarfcn", UintegerValue(100));
    lteHelper -> SetEnbDeviceAttribute("UlEarfcn", UintegerValue(18100));
    lteHelper -> SetAttribute("PathlossModel",
        StringValue("ns3::NakagamiPropagationLossModel"));

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults();

    // real RRC modeling
    // May cause lost messages
    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(false));

    //-------------Antenna Parameters
    lteHelper -> SetEnbAntennaModelType("ns3::CosineAntennaModel");
    lteHelper -> SetEnbAntennaModelAttribute("Orientation", DoubleValue(0));
    lteHelper -> SetEnbAntennaModelAttribute("Beamwidth", DoubleValue(60));
    lteHelper -> SetEnbAntennaModelAttribute("MaxGain", DoubleValue(0.0));

    // Remote host creation
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(node_remote);
    Ptr < Node > remoteHost = remoteHostContainer.Get(0);

    // Pilha de Internet
    InternetStackHelper internet;
    internet.Install(remoteHost);

    // Cria link Internet
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1400));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    // Address for the network between PGW and Remote Host
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("10.1.0.0", "255.255.0.0");
    Ipv4InterfaceContainer internetIpIfaces;
    internetIpIfaces = ipv4h.Assign(internetDevices);

    // add route from remote host to UEs
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr < Ipv4StaticRouting > remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting(remoteHost -> GetObject < Ipv4 > ());
    remoteHostStaticRouting -> AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
        Ipv4Mask("255.0.0.0"), 1);

    // Network nodes creation
    NodeContainer nodesUe;
    nodesUe.Create(node_ue);

    NodeContainer enbNodes;
    enbNodes.Create(node_enb);

    NodeContainer MECNodes;
    MECNodes.Create(node_enb);

    internet.Install(nodesUe);
    internet.Install(MECNodes);

    /*-----------------POSIÇÃO DAS TORRES----------------------------------*/
    Ptr < ListPositionAllocator > HpnPosition = CreateObject < ListPositionAllocator > ();
    ArrayPositionAllocator(HpnPosition);

    MobilityHelper mobilityEnb;
    mobilityEnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityEnb.SetPositionAllocator(HpnPosition);
    mobilityEnb.Install(enbNodes);

    // remote host mobility (constant)
    MobilityHelper remoteHostMobility;
    remoteHostMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    remoteHostMobility.Install(remoteHost);
    remoteHostMobility.Install(pgw);

    /*-----------------MONILIDADE DAS TORRES --------------*/
    // User Devices mobility
    Ns2MobilityHelper ue_mobil = Ns2MobilityHelper("mobil/mobility_25_users.tcl");
    MobilityHelper ueMobility;
    MobilityHelper enbMobility;

    ue_mobil.Install(nodesUe.Begin(), nodesUe.End());

    //-------------Instala LTE Devices para cada grupo de nós
    NetDeviceContainer enbLteDevs;
    enbLteDevs = lteHelper -> InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs;
    ueLteDevs = lteHelper -> InstallUeDevice(nodesUe);

    Ptr < Node > sgw = epcHelper -> GetSgwNode();

    Ipv4AddressHelper s1uIpv4AddressHelper;

    // Create networks of the S1 interfaces
    s1uIpv4AddressHelper.SetBase("10.0.0.0", "255.255.255.252");

    for (uint16_t i = 0; i < enbNodes.GetN(); ++i) {
        Ptr < Node > enb = enbNodes.Get(i);

        // Create a point to point link between the eNB and the SGW with
        // the corresponding new NetDevices on each side
        PointToPointHelper p2ph;
        DataRate s1uLinkDataRate = DataRate("10Gb/s");
        uint16_t s1uLinkMtu = 2000;
        Time s1uLinkDelay = Time(0);
        p2ph.SetDeviceAttribute("DataRate", DataRateValue(s1uLinkDataRate));
        p2ph.SetDeviceAttribute("Mtu", UintegerValue(s1uLinkMtu));
        p2ph.SetChannelAttribute("Delay", TimeValue(s1uLinkDelay));
        NetDeviceContainer sgwEnbDevices = p2ph.Install(sgw, enb);

        Ipv4InterfaceContainer sgwEnbIpIfaces = s1uIpv4AddressHelper.Assign(sgwEnbDevices);
        s1uIpv4AddressHelper.NewNetwork();

        Ipv4Address sgwS1uAddress = sgwEnbIpIfaces.GetAddress(0);
        Ipv4Address enbS1uAddress = sgwEnbIpIfaces.GetAddress(1);

        // Create S1 interface between the SGW and the eNB
        epcHelper -> AddS1Interface(enb, enbS1uAddress, sgwS1uAddress);
    }

    // Assign an ip for user devices
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper -> AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));

    // add routes from user devides to network gateway
    // in lte, the gateway is the pgw node, connected to the remote host
    for (uint32_t u = 0; u < nodesUe.GetN(); ++u) {
        Ptr < Node > ueNode = nodesUe.Get(u);
        Ptr < Ipv4StaticRouting > ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNode -> GetObject < Ipv4 > ());
        ueStaticRouting -> SetDefaultRoute(epcHelper -> GetUeDefaultGatewayAddress(), 1);
    }

    // attach mec servers to enodeb's
    PointToPointHelper p2pMEC;
    p2pMEC.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2pMEC.SetDeviceAttribute("Mtu", UintegerValue(1400));
    p2pMEC.SetChannelAttribute("Delay", TimeValue(Seconds(0.02)));

    NetDeviceContainer MECDevice;
    Ipv4AddressHelper address;
    address.SetBase("6.6.6.0", "255.255.255.0");

    for (uint16_t i = 0; i < MECNodes.GetN(); ++i) {
        MECDevice = p2pMEC.Install(enbNodes.Get(i), MECNodes.Get(i));
        Ipv4InterfaceContainer interfaces = address.Assign(MECDevice);
    }

    // // TODO: wrap this in a method
    uint16_t cbrPort = 3000;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    for (uint32_t u = 0; u < MECNodes.GetN(); ++u) {

        // get cbr node
        Ptr < Node > ueNode = MECNodes.Get(u);

        // get node address
        Ptr < Ipv4 > ipv4 = ueNode -> GetObject < Ipv4 > ();
        Ipv4InterfaceAddress iaddr = ipv4 -> GetAddress(1, 0);
        Ipv4Address addri = iaddr.GetLocal();

        // install server on MEC node
        PacketSinkHelper packetSinkHelper(
            "ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), cbrPort));
        serverApps.Add(packetSinkHelper.Install(ueNode));
        serverApps.Start(Seconds(2));

        // install client on remote host
        // todo: 
        int load = 1024;
        UdpClientHelper client(addri, cbrPort);
        client.SetAttribute("Interval", TimeValue(MilliSeconds(1)));
        client.SetAttribute("MaxPackets", UintegerValue(1000));
        client.SetAttribute("PacketSize", UintegerValue(load));
        clientApps.Add(client.Install(nodesUe.Get(u)));

        clientApps.Start(Seconds(3));
    }

    // attach nodes and add x2 (for signaling) interface to enbs
    lteHelper -> Attach(ueLteDevs);
    lteHelper -> AddX2Interface(enbNodes);

    /*----------------NETANIM CONFIGURATION----------------*/
    AnimationInterface anim("pandora_tmp/LTEnormal_v2x.xml");
    for (uint32_t i = 0; i < enbNodes.GetN(); ++i) {
        anim.UpdateNodeDescription(enbNodes.Get(i), "eNb");
        anim.UpdateNodeColor(enbNodes.Get(i), 0, 255, 0);
    }
    for (uint32_t i = 0; i < nodesUe.GetN(); ++i) {
        anim.UpdateNodeDescription(nodesUe.Get(i), "UE Carro");
        anim.UpdateNodeColor(nodesUe.Get(i), 255, 0, 0);
    }
    anim.UpdateNodeDescription(remoteHost, "RH");
    anim.UpdateNodeColor(remoteHost, 0, 255, 255);

    Simulator::Stop(SIMULATION_TIME_FORMAT(simTime));

    /*--------------HANDOVER NOTIFICATIONS-------------------------*/
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
        MakeCallback( & NotifyConnectionEstablishedUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
        MakeCallback( & NotifyHandoverStartUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
        MakeCallback( & NotifyHandoverEndOkUe));

    // Flow monitor
    FlowMonitorHelper flowmon;
    Ptr < FlowMonitor > monitor = flowmon.InstallAll();

    // run the simulation
    Simulator::Run();

    // Flow monitor display staistics
    monitor -> CheckForLostPackets();
    Ptr < Ipv4FlowClassifier > classifier = DynamicCast < Ipv4FlowClassifier > (flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor -> GetFlowStats();
    for (std::map < FlowId, FlowMonitor::FlowStats > ::const_iterator i = stats.begin(); i != stats.end(); ++i) {
        if (i -> first > 2) {
            Ipv4FlowClassifier::FiveTuple t = classifier -> FindFlow(i -> first);
            std::cout << "Flow " << i -> first - 2 << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
            std::cout << "  Tx Packets: " << i -> second.txPackets << "\n";
            std::cout << "  Tx Bytes:   " << i -> second.txBytes << "\n";
            std::cout << "  TxOffered:  " << i -> second.txBytes * 8.0 / 9.0 / 1000 / 1000 << " Mbps\n";
            std::cout << "  Rx Packets: " << i -> second.rxPackets << "\n";
            std::cout << "  Rx Bytes:   " << i -> second.rxBytes << "\n";
            std::cout << "  Throughput: " << i -> second.rxBytes * 8.0 / 9.0 / 1000 / 1000 << " Mbps\n";
        }
    }

    Simulator::Destroy();
    return EXIT_SUCCESS;
}
