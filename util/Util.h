#pragma once

#include "../datastructure/Ellipse.h"
#include "../datastructure/PipelineGrid.h"

namespace Util {

std::array<PipelineGrid::gridconfig_t, 2> gridCandidatesFromEllipse(const pipeline::Ellipse& ellipse, const double rotation = 0);

}
