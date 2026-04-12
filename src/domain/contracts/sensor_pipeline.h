#pragma once

namespace lightning::loc {
class ISensorPipeline;
}

namespace lightning::domain::contracts {

// Temporary stable entry point over the legacy sensor pipeline contract.
using ISensorPipeline = lightning::loc::ISensorPipeline;

}  // namespace lightning::domain::contracts
