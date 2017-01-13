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

//package fattree;

@license(LGPL);



simple Router
{
    parameters:
        //@display("i=block/routing");
    gates:
		inout port_0;
		inout port_1;
		inout port_2;
		inout port_3;
}
simple Processor
{
    parameters:
        //@display("i=block/routing");
        volatile double sendNext @unit(s);   // delay before sending next message
    gates:
		inout port;
}

network Fat_tree
{
	//@display("bgb=110,127");
    types:
        channel Channel extends ned.DatarateChannel
        {
            delay = 1ns;
            datarate = 10Gbps;
            //ber = 1e-10;//bit error rate
        }
	submodules:
        router[20]: Router {
            //@display("p=49,62");
        }
        processor[16]: Processor{
        	//@display("p=49,62");
        }
	connections:
		//processor和switch相连
		processor[0].port <--> Channel <--> router[0].port_0;
		processor[1].port <--> Channel <--> router[0].port_1;
		processor[2].port <--> Channel <--> router[1].port_0;
		processor[3].port <--> Channel <--> router[1].port_1;
		processor[4].port <--> Channel <--> router[2].port_0;
		processor[5].port <--> Channel <--> router[2].port_1;
		processor[6].port <--> Channel <--> router[3].port_0;
		processor[7].port <--> Channel <--> router[3].port_1;
		processor[8].port <--> Channel <--> router[4].port_0;
		processor[9].port <--> Channel <--> router[4].port_1;
		processor[10].port <--> Channel <--> router[5].port_0;
		processor[11].port <--> Channel <--> router[5].port_1;
		processor[12].port <--> Channel <--> router[6].port_0;
		processor[13].port <--> Channel <--> router[6].port_1;
		processor[14].port <--> Channel <--> router[7].port_0;
		processor[15].port <--> Channel <--> router[7].port_1;
		//switch和switch相连
		router[0].port_2 <--> Channel <--> router[8].port_0;
		router[0].port_3 <--> Channel <--> router[9].port_0;
		router[1].port_2 <--> Channel <--> router[8].port_1;
		router[1].port_3 <--> Channel <--> router[9].port_1;
		router[2].port_2 <--> Channel <--> router[10].port_0;
		router[2].port_3 <--> Channel <--> router[11].port_0;
		router[3].port_2 <--> Channel <--> router[10].port_1;
		router[3].port_3 <--> Channel <--> router[11].port_1;
		router[4].port_2 <--> Channel <--> router[12].port_0;
		router[4].port_3 <--> Channel <--> router[13].port_0;
		router[5].port_2 <--> Channel <--> router[12].port_1;
		router[5].port_3 <--> Channel <--> router[13].port_1;
		router[6].port_2 <--> Channel <--> router[14].port_0;
		router[6].port_3 <--> Channel <--> router[15].port_0;
		router[7].port_2 <--> Channel <--> router[14].port_1;
		router[7].port_3 <--> Channel <--> router[15].port_1;
		router[8].port_2 <--> Channel <--> router[16].port_0;
		router[8].port_3 <--> Channel <--> router[18].port_0;
		router[9].port_2 <--> Channel <--> router[17].port_0;
		router[9].port_3 <--> Channel <--> router[19].port_0;
		router[10].port_2 <--> Channel <--> router[16].port_1;
		router[10].port_3 <--> Channel <--> router[18].port_1;
		router[11].port_2 <--> Channel <--> router[17].port_1;
		router[11].port_3 <--> Channel <--> router[19].port_1;
		router[12].port_2 <--> Channel <--> router[16].port_2;
		router[12].port_3 <--> Channel <--> router[18].port_2;
		router[13].port_2 <--> Channel <--> router[17].port_2;
		router[13].port_3 <--> Channel <--> router[19].port_2;
		router[14].port_2 <--> Channel <--> router[16].port_3;
		router[14].port_3 <--> Channel <--> router[18].port_3;
		router[15].port_2 <--> Channel <--> router[17].port_3;
		router[15].port_3 <--> Channel <--> router[19].port_3;
        
}
