#include <aris_control_motion.h>
#include <iostream>
#include "ecrt.h"

int main()
{
    aris::core::XmlDocument doc;
    doc.LoadFile("/usr/Robots/resource/Robot_Type_I/Robot_III.xml");
    auto ele = doc.RootElement()->FirstChildElement("server")
        ->FirstChildElement("control")->FirstChildElement("EtherCat");

    auto pMas = aris::control::EthercatMaster::createInstance<aris::control::EthercatController>();
    std::cout<<"1"<<std::endl;	
    pMas->loadXml(std::ref(*ele));
    std::cout<<"2"<<std::endl;
    pMas->start();
    std::cout<<"3"<<std::endl;

    while (true)
    {
        aris::core::Msg msg;
        pMas->msgPipe().recvInNrt(msg);
        std::cout << "NRT msg length:" << msg.size()<<" pos:" << *reinterpret_cast<std::int32_t*>(msg.data())<<std::endl;
        //msg.SetLength(10);
        msg.copy("congratulations\n");
        //pMas->msgPipe().sendToRT(msg);
    }

    return 0;
}
