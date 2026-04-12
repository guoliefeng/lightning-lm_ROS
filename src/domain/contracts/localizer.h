#pragma once

namespace lightning::loc {
class ILocalizer;
}

namespace lightning::domain::contracts {

// Temporary stable entry point over the legacy localizer contract.
using ILocalizer = lightning::loc::ILocalizer;

}  // namespace lightning::domain::contracts
