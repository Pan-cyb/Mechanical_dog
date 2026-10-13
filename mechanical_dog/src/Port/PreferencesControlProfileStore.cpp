#include "Port/PreferencesControlProfileStore.h"
namespace dog {
namespace {constexpr const char* kNamespace="applepyprof";constexpr const char* kKey="profile";}
bool PreferencesControlProfileStore::begin(){ready_=preferences_.begin(kNamespace,false);return ready_;}
ControlProfileStoreResult PreferencesControlProfileStore::load(ControlProfileRecord& record){
    if(!ready_)return ControlProfileStoreResult::IoError;
    if(!preferences_.isKey(kKey))return ControlProfileStoreResult::Missing;
    if(preferences_.getBytesLength(kKey)!=sizeof(record))return ControlProfileStoreResult::Invalid;
    return preferences_.getBytes(kKey,&record,sizeof(record))==sizeof(record)?
        ControlProfileStoreResult::Ok:ControlProfileStoreResult::IoError;
}
ControlProfileStoreResult PreferencesControlProfileStore::save(const ControlProfileRecord& record){
    if(!ready_)return ControlProfileStoreResult::IoError;
    return preferences_.putBytes(kKey,&record,sizeof(record))==sizeof(record)?
        ControlProfileStoreResult::Ok:ControlProfileStoreResult::IoError;
}
}
