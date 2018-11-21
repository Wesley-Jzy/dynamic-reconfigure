#include "utils.hpp"
#include "uuid/uuid.h"

namespace rqt_reconfigure {

std::string Utils::get_uuid() {
    uuid_t uuid;
    uuid_generate(uuid);
    char _str[36];
    uuid_unparse(uuid, _str);
    std::string uuid_name(_str);
    uuid_name.erase(std::remove(uuid_name.begin(), uuid_name.end(), '-'), uuid_name.end());
    return uuid_name;
}

} //rqt_reconfigure namespace end

