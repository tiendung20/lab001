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

package lab001.veins_inet;

import inet.networklayer.configurator.ipv4.HostAutoConfigurator;
import inet.node.inet.AdhocHost;
import org.car2x.veins.modules.application.traci.TraCIDemo11p;
//
// Wireless-enabled Host
//
simple VeinsInetPerson extends TraCIDemo11p//extends AdhocHost
{
    parameters:
        @display("i=device/cellphone");
        @class(veins::VeinsInetPerson);
        //ipv4.configurator.networkConfiguratorModule = "";

    //submodules:
        //ifConfig: HostAutoConfigurator {
        //   @display("p=125,320;is=s");
        //}

}
