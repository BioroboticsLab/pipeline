#pragma once

#include "../datastructure/Ellipse.h"
#include "../datastructure/PipelineGrid.h"

namespace Util {

std::pair<PipelineGrid, PipelineGrid> gridCandidatesFromEllipse(const pipeline::Ellipse& ellipse, const double rotation = 0);

}
