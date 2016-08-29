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

//
// TODO generated message class
//
message FatTreeMsg{
    //int someField;
    //string anotherField;
    //double arrayField1[];
    //double arrayField2[10];
    int src_ppid; //源processor的physical id
    int dst_ppid; //目标processor的physical id
    int from_router_port; //记录从当前路由器的哪一个端口收到该msg，由上一个路由器计算出
    string anotherField;
    int hopCount = 0;
    int vc_id; //virtual channel id
}
