/**
 * @file Representations/BehaviorControl/Role.h
 *
 * Declaration of the representation of a robot's behavior role
 *
 * @author Tim Laue, Andreas Stolpmann
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Role.h"
#include "Representations/Communication/NaovaTeamMessageParts/NaovaMessageParticule.h"


STREAMABLE(TeammateRoles, COMMA public NaovaMessageParticule<idTeammateRoles>
{
  /** NaovaMessageParticle functions */
  void operator >> (NaovaMessage& m) const override;
  void operator << (const NaovaMessage& m) override;

  Role::RoleType operator [](const size_t i) const;
  Role::RoleType& operator [](const size_t i);
  static const char* getName(Role::RoleType e),

  (std::vector<Role::RoleType>) roles,
});
