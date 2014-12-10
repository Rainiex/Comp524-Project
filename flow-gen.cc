/*
 *Generate varied flows used in experiemnt 5
 *
 *by Jie
 */

#include "ns3/core-module.h"
#include <iostream>
#include <cmath>
#include <string>
#include <sstream>
#include "ns3/random-variable.h"
#include "ns3/rng-seed-manager.h"

using namespace ns3;

int main (int argc, char **argv){
    int i,j,k;
	double expMean = 0.2;
	double totalTime = 2;
    double expMean2 = 0.2;
    double totalTime2 = 2;
    RngSeedManager::SetSeed (3);
    RngSeedManager::SetRun (7);
    
    for(i=1; i<=4; i++){
        std::cout << "traffic" << i << ":" <<std::endl;
        std::cout << "DL: " << std::endl;
        for(j=1; j<=5; j++){
            std::cout << "Flow" << j << " Ontime: 0.2s ";
            
            Ptr<ExponentialRandomVariable> ev = CreateObject<ExponentialRandomVariable> ();
            ev->SetAttribute ("Mean", DoubleValue (expMean));
            ev->SetAttribute ("Bound", DoubleValue (totalTime));
            
            Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
            uv->SetAttribute ("Min", DoubleValue (0.5));
            uv->SetAttribute ("Max", DoubleValue (4));
            
            std::cout << "Offtime: " << ev->GetValue() << "s DataRate: " << uv->GetValue() << "Mbps" <<std::endl;
        }
        
        std::cout << "UL: " << std::endl;
        for(k=1; k<=5; k++){
            std::cout << "Flow" << k << " Ontime: 0.2s ";
            
            Ptr<ExponentialRandomVariable> ev = CreateObject<ExponentialRandomVariable> ();
            ev->SetAttribute ("Mean", DoubleValue (expMean2));
            ev->SetAttribute ("Bound", DoubleValue (totalTime2));
            
            Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
            uv->SetAttribute ("Min", DoubleValue (0.5));
            uv->SetAttribute ("Max", DoubleValue (2));
            
            std::cout << "Offtime: " << ev->GetValue() << "s DataRate: " << uv->GetValue() << "Mbps" <<std::endl;
        }
        
        expMean2 += 0.2;
        totalTime2 += 2;
        
        std::cout << "--------------------------------------------------" <<std::endl;
    }
    
	return 0;
}

