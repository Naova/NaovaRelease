/**
 * @file RoleChanges.h
 */

#pragma once
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Role.h"

STREAMABLE(RoleChanges,
{   
    , 
    ((Role) RoleType)(undefined) nextRole,
    (int) (0) lastRoleChange,
});