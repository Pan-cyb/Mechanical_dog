#pragma once
#include <Preferences.h>
#include "Interface/IControlProfileStore.h"
namespace dog {
class PreferencesControlProfileStore final:public IControlProfileStore{
public:
    bool begin();
    ControlProfileStoreResult load(ControlProfileRecord& record)override;
    ControlProfileStoreResult save(const ControlProfileRecord& record)override;
private:Preferences preferences_;bool ready_=false;
};
}
