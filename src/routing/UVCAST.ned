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

package uvcast.routing;

import org.car2x.veins.modules.application.ieee80211p.BaseWaveApplLayer;

//
// TODO auto-generated module
//
simple UVCAST extends BaseWaveApplLayer
{
    parameters:
        @statistic[lastTxPower](record=mean);
    	@statistic[meanSNR](record=mean); 
        @statistic[sentDownMACInCCH](record=sum); 
        @statistic[collisions](record=sum);
        @statistic[messagesTransmittedSCF](record=sum);
        @statistic[messagesReceivedSCF](record=sum);
        @statistic[duplicatedMessages](record=sum);
        @statistic[messagesTransmitted](record=sum);
        @statistic[messagesReceived](record=sum);
        @statistic[isInROI](record=sum);
        
        // logging parameters
        int log_traffic;
        int log_replication;
                      
        // Event information. Since in our scenarios the RSU starts the dissemination,
        // these parameters should be the same as for the RSU
        double startDataProductionTime @unit(s) = default(0s);
        double dataTTL @unit(s) = default(90s);
        double dataROI @unit(m) = default(0m);        
        double eventOriginX = default(0);
        double eventOriginY = default(0);
        double eventOriginZ = default(0);        
        
        // Back-traffic application. Used to simulate a scenario in which the dissemination
        // must share the channel with other applications
        bool generateBackTraffic = default(false);
        
        // Time interval to delete out-dated received beacons
        double beaconEntryTTL @unit(s) = default(3s);
        
        double avgCommRange @unit(m) = default(500m);
        double maxGPSError @unit(m) = default(0m);
        
        double maxDelay @unit(s) = default(500ms);                
        
        bool desynchronize = default(false);  
        
        bool adaptTxPower = default(false);                                            
            	
 		@class(UVCAST);   	
}

