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
#include "ns3/netanim-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/lte-module.h"
#include "ns3/string.h"
#include "ns3/double.h"
#include <ns3/boolean.h>
#include <ns3/enum.h>
#include "ns3/output-stream-wrapper.h"
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

const int number_of_ue = 1;

const uint16_t number_of_mc = 7;

const int node_enb = number_of_mc;

uint16_t n_cbr = useCbr ? number_of_mc: 0;

int mcTxPower = 40;

double simTime = 1;

unsigned int handNumber = 0;

bool randomCellAlloc = true;
bool rowTopology = false;

//coeficiente da média exponencial
unsigned int exp_mean_window = 3;
double qosLastValue = 0;
double qoeLastValue = 0;

int cell_ue[number_of_mc][number_of_ue];

NS_LOG_COMPONENT_DEFINE("pandora");

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

    for (uint16_t i =0; i < number_of_mc; ++i)
    {
        ::cell_ue[i][imsi - 1] = 0;
    }

    ::cell_ue[cellid - 1][imsi - 1] = rnti;
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
    std::ifstream pos_file("CellsDataset");
    int cellId;
    double x_coord, y_coord;
    char a;

    while (pos_file >> cellId >> x_coord >> y_coord >> a)
    {
      NS_LOG_INFO("Adding cell " << cellId <<
        " to coordinates x:" << x_coord << " y:" << y_coord);
      HpnPosition->Add(Vector(x_coord, y_coord, 30));
    }
}

uint16_t getServingCellId(uint16_t ueId)
{
    uint16_t servingCellId = 0;
    for (uint16_t i = 0; i < number_of_mc; ++i)
    {
        if (::cell_ue[i][ueId - 1] != 0)
            servingCellId = i + 1;
    }
    return servingCellId;
}

void executeHandover(Ptr <LteHelper> lteHelper, 
    uint16_t imsi, 
    uint16_t targetCellId,
    NetDeviceContainer ueLteDevs,
    NetDeviceContainer enbLteDevs
    )
{
    NS_LOG_UNCOND("user: " << imsi);
    NS_LOG_UNCOND("cell: " << getServingCellId(imsi));
    lteHelper->HandoverRequest(Simulator::Now(), ueLteDevs.Get(imsi - 1), enbLteDevs.Get(getServingCellId(imsi)), enbLteDevs.Get(targetCellId - 1));
}

void checkDelay(Ptr < Node > ueNode,
    Ptr < Node > MECNode, 
    Ipv4Address ueAddress
    )
{
  UdpEchoServerHelper echoServer (9);

  ApplicationContainer serverApps = echoServer.Install (ueNode);
  serverApps.Start (Simulator::Now());
  serverApps.Stop (Simulator::Now() + Seconds(0.5));

  UdpEchoClientHelper echoClient (ueAddress, 9);
  echoClient.SetAttribute ("MaxPackets", UintegerValue (1));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (1.0)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (1024));

  ApplicationContainer clientApps = echoClient.Install (MECNode);
  clientApps.Start (Simulator::Now());
  clientApps.Stop (Simulator::Now() + Seconds(0.1));
}

void ShowTime() {
    NS_LOG_UNCOND(Simulator::Now().GetSeconds());
    Simulator::Schedule(Seconds(0.5), & ShowTime);
}

void requestService(Ptr < Node > ueNode, Ptr < Node > MECNode, Ipv4Address ueAddress) {
    Time interPacketInterval = MilliSeconds(10);
    // Install and start applications on UEs and remote host
    uint16_t dlPort = 1100;
    uint16_t ulPort = 2000;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    serverApps.Add(dlPacketSinkHelper.Install(ueNode));

    UdpClientHelper dlClient(ueAddress, dlPort);
    dlClient.SetAttribute("Interval", TimeValue(interPacketInterval));
    dlClient.SetAttribute("MaxPackets", UintegerValue(1000000));
    clientApps.Add(dlClient.Install(MECNode));

    ++ulPort;
    PacketSinkHelper ulPacketSinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), ulPort));
    serverApps.Add(ulPacketSinkHelper.Install(MECNode));

    Ptr < Ipv4 > ipv4 = MECNode->GetObject < Ipv4 > ();
    Ipv4InterfaceAddress iaddr = ipv4->GetAddress(1, 0);
    Ipv4Address addri = iaddr.GetLocal();
    NS_LOG_UNCOND(addri);
    UdpClientHelper ulClient(addri, ulPort);
    ulClient.SetAttribute("Interval", TimeValue(interPacketInterval));
    ulClient.SetAttribute("MaxPackets", UintegerValue(1000000));
    clientApps.Add(ulClient.Install(ueNode));

    serverApps.Start(MilliSeconds(500));
    clientApps.Start(MilliSeconds(500));
}

// todo
void migrateService() {}

int
main(int argc, char * argv[]) {
    Time simTime = MilliSeconds(1900);
    Time interPacketInterval = MilliSeconds(100);

    LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
  LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults();

    LogComponentEnable("pandora", LOG_LEVEL_ALL);

    // Bandwidth of Dl and Ul in Resource Blocks
    Config::SetDefault("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(25));
    Config::SetDefault("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(25));

    // Modo de transmissão (SISO [0], MIMO [1])
    Config::SetDefault("ns3::LteEnbRrc::DefaultTransmissionMode",
        UintegerValue(1));

    Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(320));

    // ------------------------- lte and epc helper ----------------------
    Ptr < LteHelper > lteHelper = CreateObject < LteHelper > ();
    Ptr < EpcHelper > epcHelper;
    epcHelper = CreateObject < NoBackhaulEpcHelper > ();
    lteHelper->SetEpcHelper(epcHelper);

    // LTE configuration
    lteHelper->SetSchedulerType("ns3::PssFfMacScheduler");
    lteHelper->SetSchedulerAttribute("nMux", UintegerValue(1)); // the maximum number of UE selected by TD scheduler
    lteHelper->SetSchedulerAttribute("PssFdSchedulerType", StringValue("CoItA")); // PF scheduler type in PSS
    lteHelper->SetHandoverAlgorithmType("ns3::A2A4RsrqHandoverAlgorithm");
    lteHelper->SetHandoverAlgorithmAttribute("ServingCellThreshold",
    UintegerValue(34));
    lteHelper->SetHandoverAlgorithmAttribute("NeighbourCellOffset",
    UintegerValue(0));
    lteHelper->EnableTraces();

    // Propagation Parameters
    lteHelper->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(100));
    lteHelper->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(18100));
    lteHelper->SetAttribute("PathlossModel",
        StringValue("ns3::NakagamiPropagationLossModel"));

    //-------------Antenna Parameters
    lteHelper->SetEnbAntennaModelType("ns3::CosineAntennaModel");
    lteHelper->SetEnbAntennaModelAttribute("Orientation", DoubleValue(0));
    lteHelper->SetEnbAntennaModelAttribute("Beamwidth", DoubleValue(60));
    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(false));

    Ptr < Node > pgw = epcHelper->GetPgwNode();

    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create(number_of_mc);
    ueNodes.Create(number_of_ue);

    NodeContainer MecContainer;
    MecContainer.Create(number_of_mc);

    // Create a single RemoteHost
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr < Node > remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);
    internet.Install(MecContainer);

    // Create the Internet
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(0)));

    NetDeviceContainer MecDevices;
    NetDeviceContainer PgwDevice;
    for (uint16_t u = 0; u < number_of_mc; u++) {
        NetDeviceContainer c = p2ph.Install(pgw, MecContainer.Get(u));
        PgwDevice.Add(c.Get(0));
        MecDevices.Add(c.Get(1));
    }

    Ipv4InterfaceContainer PgwInterfaces;
    Ipv4InterfaceContainer MecInterfaces;

    //Ipv4AddressHelper pgwRouterIpv4 ("192.168.0.0", "255.255.0.0");
    //Ipv4InterfaceContainer pgwRouterInterfaces = pgwRouterIpv4.Assign(pgwRouterDevices);

    //Ipv4AddressHelper internetIpv4 ("1.0.0.0", "255.0.0.0");
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    for (uint16_t r = 0; r < number_of_mc; ++r) {
        NetDeviceContainer ndc;
        ndc.Add(PgwDevice.Get(r));
        ndc.Add(MecDevices.Get(r));
        Ipv4InterfaceContainer ifc = ipv4h.Assign(ndc);
        PgwInterfaces.Add(ifc.Get(0));
        MecInterfaces.Add(ifc.Get(1));
    }

    Ipv4StaticRoutingHelper ipv4RoutingHelper;

    uint16_t j = 1;
    for (uint16_t u = 0; u < number_of_mc; u++) {
        std::stringstream ss;
        ss << u + j;
        Ptr < Ipv4StaticRouting > MecRouting = ipv4RoutingHelper.GetStaticRouting(MecContainer.Get(u)->GetObject < Ipv4 > ());
        MecRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), Ipv4Address(("1.0.0." + ss.str()).c_str()), 1);
        j++;
    }

    Ptr < Ipv4StaticRouting > PgwRouting = ipv4RoutingHelper.GetStaticRouting(pgw->GetObject < Ipv4 > ());
    //PgwRouting->AddNetworkRouteTo (Ipv4Address ("1.0.0.0"),Ipv4Mask ("255.0.0.0"),1);
    j = 2;
    for (uint16_t u = 0; u < number_of_mc; u++) {
        std::stringstream ss;
        ss << u + j;
        PgwRouting->AddHostRouteTo(Ipv4Address(("1.0.0." + ss.str()).c_str()), j + 1);
        j++;
    }

    /*-----------------POSITION AND MOBILITY----------------------------------*/
    Ptr < ListPositionAllocator > HpnPosition = CreateObject < ListPositionAllocator > ();
    ArrayPositionAllocator(HpnPosition);

    MobilityHelper mobilityEnb;
    mobilityEnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityEnb.SetPositionAllocator(HpnPosition);
    mobilityEnb.Install(enbNodes);

    // remote host mobility (constant)
    MobilityHelper remoteHostMobility;
    remoteHostMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    remoteHostMobility.Install(pgw);
    remoteHostMobility.Install(MecContainer);

    // User Devices mobility
    Ns2MobilityHelper ue_mobil = Ns2MobilityHelper("mobil/5961.tcl");
    MobilityHelper ueMobility;
    MobilityHelper enbMobility;

    ue_mobil.Install(ueNodes.Begin(), ueNodes.End());

    // SGW node
    Ptr < Node > sgw = epcHelper->GetSgwNode();

    // Install Mobility Model for SGW
    Ptr < ListPositionAllocator > positionAlloc2 = CreateObject < ListPositionAllocator > ();
    positionAlloc2->Add(Vector(0.0, 50.0, 0.0));
    MobilityHelper mobility2;
    mobility2.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility2.SetPositionAllocator(positionAlloc2);
    mobility2.Install(sgw);

    // Install LTE Devices to the nodes
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);
    //Ipv4Address addr = remoteHostAddr;
    Ipv4AddressHelper s1uIpv4AddressHelper;

    // Create networks of the S1 interfaces
    s1uIpv4AddressHelper.SetBase("10.0.0.0", "255.255.255.252");

    for (uint16_t i = 0; i < number_of_mc; ++i) {
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
        //addr = enbS1uAddress;

        // Create S1 interface between the SGW and the eNB
        epcHelper->AddS1Interface(enb, enbS1uAddress, sgwS1uAddress);
    }

    // Install the IP stack on the UEs
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));
    // Assign IP address to UEs, and install applications
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u) {
        Ptr < Node > ueNode = ueNodes.Get(u);
        // Set the default gateway for the UE
        Ptr < Ipv4StaticRouting > ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject < Ipv4 > ());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    lteHelper->Attach(ueLteDevs);

    // checkDelay(ueNodes.Get(0), MecContainer.Get(0), ueIpIface.GetAddress(0));
    for (uint16_t i = 1; i < 10; ++i)
        Simulator::Schedule(Seconds(i), &checkDelay, ueNodes.Get(0), MecContainer.Get(0), ueIpIface.GetAddress(0));

    for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      requestService(ueNodes.Get(u), MecContainer.Get(u), ueIpIface.GetAddress(u));
    }

    Ptr < OutputStreamWrapper > routingStream = Create < OutputStreamWrapper >
        ("static-global-routing.routes", std::ios::out);
    ipv4RoutingHelper.PrintRoutingTableAllAt(Seconds(1), routingStream);

    AnimationInterface anim("pandora_anim.xml");
    for (uint32_t i = 0; i < enbNodes.GetN(); ++i) {
        anim.UpdateNodeDescription(enbNodes.Get(i), "eNb--------------");
        anim.UpdateNodeColor(enbNodes.Get(i), 0, 255, 0);
    }
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i) {
        anim.UpdateNodeDescription(ueNodes.Get(i), "UE");
        anim.UpdateNodeColor(ueNodes.Get(i), 255, 0, 0);
    }
    Simulator::Schedule(Seconds(1), &ShowTime);

    FlowMonitorHelper flowmon;
    Ptr < FlowMonitor > monitor = flowmon.InstallAll();
    // Uncomment to enable PCAP tracing
    //p2ph.EnablePcapAll("lena-simple-epc-backhaul");

    Simulator::Stop(Seconds(10));

    /*--------------HANDOVER NOTIFICATIONS-------------------------*/
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
        MakeCallback( & NotifyConnectionEstablishedUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
        MakeCallback( & NotifyHandoverStartUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
        MakeCallback( & NotifyHandoverEndOkUe));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr < Ipv4FlowClassifier > classifier = DynamicCast < Ipv4FlowClassifier > (flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
    for (std::map < FlowId, FlowMonitor::FlowStats > ::const_iterator i = stats.begin(); i != stats.end(); ++i) {
        if (i->first > 2) {
            Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
            std::cout << "Flow " << i->first - 2 << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
            std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
            std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
            std::cout << "  TxOffered:  " << i->second.txBytes * 8.0 / 9.0 / 1000 / 1000 << " Mbps\n";
            std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
            std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
            std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / 9.0 / 1000 / 1000 << " Mbps\n";
        }
    }

    Simulator::Destroy();
    return 0;
}
