//
// Generated file, do not edit! Created by nedtool 5.0 from fat_tree_pkt.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "fat_tree_pkt_m.h"

namespace omnetpp {

// Template pack/unpack rules. They are declared *after* a1l type-specific pack functions for multiple reasons.
// They are in the omnetpp namespace, to allow them to be found by argument-dependent lookup via the cCommBuffer argument

// Packing/unpacking an std::vector
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::vector<T,A>& v)
{
    int n = v.size();
    doParsimPacking(buffer, n);
    for (int i = 0; i < n; i++)
        doParsimPacking(buffer, v[i]);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::vector<T,A>& v)
{
    int n;
    doParsimUnpacking(buffer, n);
    v.resize(n);
    for (int i = 0; i < n; i++)
        doParsimUnpacking(buffer, v[i]);
}

// Packing/unpacking an std::list
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::list<T,A>& l)
{
    doParsimPacking(buffer, (int)l.size());
    for (typename std::list<T,A>::const_iterator it = l.begin(); it != l.end(); ++it)
        doParsimPacking(buffer, (T&)*it);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::list<T,A>& l)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        l.push_back(T());
        doParsimUnpacking(buffer, l.back());
    }
}

// Packing/unpacking an std::set
template<typename T, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::set<T,Tr,A>& s)
{
    doParsimPacking(buffer, (int)s.size());
    for (typename std::set<T,Tr,A>::const_iterator it = s.begin(); it != s.end(); ++it)
        doParsimPacking(buffer, *it);
}

template<typename T, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::set<T,Tr,A>& s)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        T x;
        doParsimUnpacking(buffer, x);
        s.insert(x);
    }
}

// Packing/unpacking an std::map
template<typename K, typename V, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::map<K,V,Tr,A>& m)
{
    doParsimPacking(buffer, (int)m.size());
    for (typename std::map<K,V,Tr,A>::const_iterator it = m.begin(); it != m.end(); ++it) {
        doParsimPacking(buffer, it->first);
        doParsimPacking(buffer, it->second);
    }
}

template<typename K, typename V, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::map<K,V,Tr,A>& m)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        K k; V v;
        doParsimUnpacking(buffer, k);
        doParsimUnpacking(buffer, v);
        m[k] = v;
    }
}

// Default pack/unpack function for arrays
template<typename T>
void doParsimArrayPacking(omnetpp::cCommBuffer *b, const T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimPacking(b, t[i]);
}

template<typename T>
void doParsimArrayUnpacking(omnetpp::cCommBuffer *b, T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimUnpacking(b, t[i]);
}

// Default rule to prevent compiler from choosing base class' doParsimPacking() function
template<typename T>
void doParsimPacking(omnetpp::cCommBuffer *, const T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: no doParsimPacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

template<typename T>
void doParsimUnpacking(omnetpp::cCommBuffer *, T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: no doParsimUnpacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

}  // namespace omnetpp


// forward
template<typename T, typename A>
std::ostream& operator<<(std::ostream& out, const std::vector<T,A>& vec);

// Template rule which fires if a struct or class doesn't have operator<<
template<typename T>
inline std::ostream& operator<<(std::ostream& out,const T&) {return out;}

// operator<< for std::vector<T>
template<typename T, typename A>
inline std::ostream& operator<<(std::ostream& out, const std::vector<T,A>& vec)
{
    out.put('{');
    for(typename std::vector<T,A>::const_iterator it = vec.begin(); it != vec.end(); ++it)
    {
        if (it != vec.begin()) {
            out.put(','); out.put(' ');
        }
        out << *it;
    }
    out.put('}');
    
    char buf[32];
    sprintf(buf, " (size=%u)", (unsigned int)vec.size());
    out.write(buf, strlen(buf));
    return out;
}

Register_Class(FatTreePkt);

FatTreePkt::FatTreePkt(const char *name, int kind) : ::omnetpp::cPacket(name,kind)
{
    this->flitCount = 0;
    this->src_ppid = 0;
    this->dst_ppid = 0;
    this->isHead = false;
    this->isTail = false;
    this->vc_id = 0;
    this->hopCount = 0;
    this->from_router_port = 0;
    this->packageGenTime = 0;
}

FatTreePkt::FatTreePkt(const FatTreePkt& other) : ::omnetpp::cPacket(other)
{
    copy(other);
}

FatTreePkt::~FatTreePkt()
{
}

FatTreePkt& FatTreePkt::operator=(const FatTreePkt& other)
{
    if (this==&other) return *this;
    ::omnetpp::cPacket::operator=(other);
    copy(other);
    return *this;
}

void FatTreePkt::copy(const FatTreePkt& other)
{
    this->flitCount = other.flitCount;
    this->src_ppid = other.src_ppid;
    this->dst_ppid = other.dst_ppid;
    this->isHead = other.isHead;
    this->isTail = other.isTail;
    this->vc_id = other.vc_id;
    this->hopCount = other.hopCount;
    this->from_router_port = other.from_router_port;
    this->packageGenTime = other.packageGenTime;
}

void FatTreePkt::parsimPack(omnetpp::cCommBuffer *b) const
{
    ::omnetpp::cPacket::parsimPack(b);
    doParsimPacking(b,this->flitCount);
    doParsimPacking(b,this->src_ppid);
    doParsimPacking(b,this->dst_ppid);
    doParsimPacking(b,this->isHead);
    doParsimPacking(b,this->isTail);
    doParsimPacking(b,this->vc_id);
    doParsimPacking(b,this->hopCount);
    doParsimPacking(b,this->from_router_port);
    doParsimPacking(b,this->packageGenTime);
}

void FatTreePkt::parsimUnpack(omnetpp::cCommBuffer *b)
{
    ::omnetpp::cPacket::parsimUnpack(b);
    doParsimUnpacking(b,this->flitCount);
    doParsimUnpacking(b,this->src_ppid);
    doParsimUnpacking(b,this->dst_ppid);
    doParsimUnpacking(b,this->isHead);
    doParsimUnpacking(b,this->isTail);
    doParsimUnpacking(b,this->vc_id);
    doParsimUnpacking(b,this->hopCount);
    doParsimUnpacking(b,this->from_router_port);
    doParsimUnpacking(b,this->packageGenTime);
}

int FatTreePkt::getFlitCount() const
{
    return this->flitCount;
}

void FatTreePkt::setFlitCount(int flitCount)
{
    this->flitCount = flitCount;
}

int FatTreePkt::getSrc_ppid() const
{
    return this->src_ppid;
}

void FatTreePkt::setSrc_ppid(int src_ppid)
{
    this->src_ppid = src_ppid;
}

int FatTreePkt::getDst_ppid() const
{
    return this->dst_ppid;
}

void FatTreePkt::setDst_ppid(int dst_ppid)
{
    this->dst_ppid = dst_ppid;
}

bool FatTreePkt::getIsHead() const
{
    return this->isHead;
}

void FatTreePkt::setIsHead(bool isHead)
{
    this->isHead = isHead;
}

bool FatTreePkt::getIsTail() const
{
    return this->isTail;
}

void FatTreePkt::setIsTail(bool isTail)
{
    this->isTail = isTail;
}

int FatTreePkt::getVc_id() const
{
    return this->vc_id;
}

void FatTreePkt::setVc_id(int vc_id)
{
    this->vc_id = vc_id;
}

int FatTreePkt::getHopCount() const
{
    return this->hopCount;
}

void FatTreePkt::setHopCount(int hopCount)
{
    this->hopCount = hopCount;
}

int FatTreePkt::getFrom_router_port() const
{
    return this->from_router_port;
}

void FatTreePkt::setFrom_router_port(int from_router_port)
{
    this->from_router_port = from_router_port;
}

long FatTreePkt::getPackageGenTime() const
{
    return this->packageGenTime;
}

void FatTreePkt::setPackageGenTime(long packageGenTime)
{
    this->packageGenTime = packageGenTime;
}

class FatTreePktDescriptor : public omnetpp::cClassDescriptor
{
  private:
    mutable const char **propertynames;
  public:
    FatTreePktDescriptor();
    virtual ~FatTreePktDescriptor();

    virtual bool doesSupport(omnetpp::cObject *obj) const override;
    virtual const char **getPropertyNames() const override;
    virtual const char *getProperty(const char *propertyname) const override;
    virtual int getFieldCount() const override;
    virtual const char *getFieldName(int field) const override;
    virtual int findField(const char *fieldName) const override;
    virtual unsigned int getFieldTypeFlags(int field) const override;
    virtual const char *getFieldTypeString(int field) const override;
    virtual const char **getFieldPropertyNames(int field) const override;
    virtual const char *getFieldProperty(int field, const char *propertyname) const override;
    virtual int getFieldArraySize(void *object, int field) const override;

    virtual std::string getFieldValueAsString(void *object, int field, int i) const override;
    virtual bool setFieldValueAsString(void *object, int field, int i, const char *value) const override;

    virtual const char *getFieldStructName(int field) const override;
    virtual void *getFieldStructValuePointer(void *object, int field, int i) const override;
};

Register_ClassDescriptor(FatTreePktDescriptor);

FatTreePktDescriptor::FatTreePktDescriptor() : omnetpp::cClassDescriptor("FatTreePkt", "omnetpp::cPacket")
{
    propertynames = nullptr;
}

FatTreePktDescriptor::~FatTreePktDescriptor()
{
    delete[] propertynames;
}

bool FatTreePktDescriptor::doesSupport(omnetpp::cObject *obj) const
{
    return dynamic_cast<FatTreePkt *>(obj)!=nullptr;
}

const char **FatTreePktDescriptor::getPropertyNames() const
{
    if (!propertynames) {
        static const char *names[] = {  nullptr };
        omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
        const char **basenames = basedesc ? basedesc->getPropertyNames() : nullptr;
        propertynames = mergeLists(basenames, names);
    }
    return propertynames;
}

const char *FatTreePktDescriptor::getProperty(const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : nullptr;
}

int FatTreePktDescriptor::getFieldCount() const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 9+basedesc->getFieldCount() : 9;
}

unsigned int FatTreePktDescriptor::getFieldTypeFlags(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeFlags(field);
        field -= basedesc->getFieldCount();
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
    };
    return (field>=0 && field<9) ? fieldTypeFlags[field] : 0;
}

const char *FatTreePktDescriptor::getFieldName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldName(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldNames[] = {
        "flitCount",
        "src_ppid",
        "dst_ppid",
        "isHead",
        "isTail",
        "vc_id",
        "hopCount",
        "from_router_port",
        "packageGenTime",
    };
    return (field>=0 && field<9) ? fieldNames[field] : nullptr;
}

int FatTreePktDescriptor::findField(const char *fieldName) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount() : 0;
    if (fieldName[0]=='f' && strcmp(fieldName, "flitCount")==0) return base+0;
    if (fieldName[0]=='s' && strcmp(fieldName, "src_ppid")==0) return base+1;
    if (fieldName[0]=='d' && strcmp(fieldName, "dst_ppid")==0) return base+2;
    if (fieldName[0]=='i' && strcmp(fieldName, "isHead")==0) return base+3;
    if (fieldName[0]=='i' && strcmp(fieldName, "isTail")==0) return base+4;
    if (fieldName[0]=='v' && strcmp(fieldName, "vc_id")==0) return base+5;
    if (fieldName[0]=='h' && strcmp(fieldName, "hopCount")==0) return base+6;
    if (fieldName[0]=='f' && strcmp(fieldName, "from_router_port")==0) return base+7;
    if (fieldName[0]=='p' && strcmp(fieldName, "packageGenTime")==0) return base+8;
    return basedesc ? basedesc->findField(fieldName) : -1;
}

const char *FatTreePktDescriptor::getFieldTypeString(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeString(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldTypeStrings[] = {
        "int",
        "int",
        "int",
        "bool",
        "bool",
        "int",
        "int",
        "int",
        "long",
    };
    return (field>=0 && field<9) ? fieldTypeStrings[field] : nullptr;
}

const char **FatTreePktDescriptor::getFieldPropertyNames(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldPropertyNames(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

const char *FatTreePktDescriptor::getFieldProperty(int field, const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldProperty(field, propertyname);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

int FatTreePktDescriptor::getFieldArraySize(void *object, int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldArraySize(object, field);
        field -= basedesc->getFieldCount();
    }
    FatTreePkt *pp = (FatTreePkt *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string FatTreePktDescriptor::getFieldValueAsString(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldValueAsString(object,field,i);
        field -= basedesc->getFieldCount();
    }
    FatTreePkt *pp = (FatTreePkt *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getFlitCount());
        case 1: return long2string(pp->getSrc_ppid());
        case 2: return long2string(pp->getDst_ppid());
        case 3: return bool2string(pp->getIsHead());
        case 4: return bool2string(pp->getIsTail());
        case 5: return long2string(pp->getVc_id());
        case 6: return long2string(pp->getHopCount());
        case 7: return long2string(pp->getFrom_router_port());
        case 8: return long2string(pp->getPackageGenTime());
        default: return "";
    }
}

bool FatTreePktDescriptor::setFieldValueAsString(void *object, int field, int i, const char *value) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->setFieldValueAsString(object,field,i,value);
        field -= basedesc->getFieldCount();
    }
    FatTreePkt *pp = (FatTreePkt *)object; (void)pp;
    switch (field) {
        case 0: pp->setFlitCount(string2long(value)); return true;
        case 1: pp->setSrc_ppid(string2long(value)); return true;
        case 2: pp->setDst_ppid(string2long(value)); return true;
        case 3: pp->setIsHead(string2bool(value)); return true;
        case 4: pp->setIsTail(string2bool(value)); return true;
        case 5: pp->setVc_id(string2long(value)); return true;
        case 6: pp->setHopCount(string2long(value)); return true;
        case 7: pp->setFrom_router_port(string2long(value)); return true;
        case 8: pp->setPackageGenTime(string2long(value)); return true;
        default: return false;
    }
}

const char *FatTreePktDescriptor::getFieldStructName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructName(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    };
}

void *FatTreePktDescriptor::getFieldStructValuePointer(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructValuePointer(object, field, i);
        field -= basedesc->getFieldCount();
    }
    FatTreePkt *pp = (FatTreePkt *)object; (void)pp;
    switch (field) {
        default: return nullptr;
    }
}


