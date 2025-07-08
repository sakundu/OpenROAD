/////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2024, Precision Innovations Inc.
// All rights reserved.
//
// BSD 3-Clause License
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////

#include "sa2d/MakeSA2D.h"
#include "sa2d/SA2D.h"
#include "ord/OpenRoad.hh"
#include "utl/Logger.h"
#include "utl/decode.h"

namespace sa2d {
// Tcl files encoded into strings.
extern const char* sa2d_tcl_inits[];
}  // namespace sa2d

extern "C" {
extern int Sa2d_Init(Tcl_Interp* interp);
}

namespace ord {

using sa2d::SA2D;

SA2D* makeSA2D()
{
  return new SA2D();
}

void deleteSA2D(SA2D* sa2d)
{
  delete sa2d;
}

void initSA2D(SA2D* sa2d,
              odb::dbDatabase* db,
              utl::Logger* logger,
              dpl::Opendp* dpl)
{
  sa2d->init(db, logger);
  sa2d->setDplEngine(dpl);
}

void initSA2D(OpenRoad* openroad)
{
  Tcl_Interp* tcl_interp = openroad->tclInterp();
  // Define swig TCL commands.
  Sa2d_Init(tcl_interp);
  // Eval encoded sa2d TCL sources.
  utl::evalTclInit(tcl_interp, sa2d::sa2d_tcl_inits);
  openroad->getSA2D()->init(openroad->getDb(), openroad->getLogger());
  openroad->getSA2D()->setDplEngine(openroad->getOpendp());
}

}  // namespace ord 