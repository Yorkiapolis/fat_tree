//
// Generated file, do not edit! Created by nedtool 5.0 from buffer_info.msg.
//

#ifndef __BUFFER_INFO_M_H
#define __BUFFER_INFO_M_H

#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0500
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif



/**
 * Class generated from <tt>buffer_info.msg:15</tt> by nedtool.
 * <pre>
 * //
 * // This program is free software: you can redistribute it and/or modify
 * // it under the terms of the GNU Lesser General Public License as published by
 * // the Free Software Foundation, either version 3 of the License, or
 * // (at your option) any later version.
 * // 
 * // This program is distributed in the hope that it will be useful,
 * // but WITHOUT ANY WARRANTY; without even the implied warranty of
 * // MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * // GNU Lesser General Public License for more details.
 * // 
 * // You should have received a copy of the GNU Lesser General Public License
 * // along with this program.  If not, see http://www.gnu.org/licenses/.
 * //
 * message BufferInfoMsg
 * {
 *     int from_port; //该msg从Router的哪个端口过来
 *     //bool BufferAvail[]; //保存与from_port端口相连的路由器对应端口的buffer状态，可变长度数组
 *     int vcid; //from upstream router, decrement credit count
 * 
 * }
 * </pre>
 */
class BufferInfoMsg : public ::omnetpp::cMessage
{
  protected:
    int from_port;
    int vcid;

  private:
    void copy(const BufferInfoMsg& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const BufferInfoMsg&);

  public:
    BufferInfoMsg(const char *name=nullptr, int kind=0);
    BufferInfoMsg(const BufferInfoMsg& other);
    virtual ~BufferInfoMsg();
    BufferInfoMsg& operator=(const BufferInfoMsg& other);
    virtual BufferInfoMsg *dup() const {return new BufferInfoMsg(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b);

    // field getter/setter methods
    virtual int getFrom_port() const;
    virtual void setFrom_port(int from_port);
    virtual int getVcid() const;
    virtual void setVcid(int vcid);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const BufferInfoMsg& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, BufferInfoMsg& obj) {obj.parsimUnpack(b);}


#endif // ifndef __BUFFER_INFO_M_H

