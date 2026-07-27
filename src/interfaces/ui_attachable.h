#pragma once

#include <memory>

namespace lightning::ui {
class PangolinWindow;
}

namespace lightning::loc {

/// 可选接口：将运行时 Pangolin UI 挂到算法模块（地图点云、LIO 扫描等）。
class IUiAttachable {
   public:
    virtual ~IUiAttachable() = default;
    virtual void AttachUi(const std::shared_ptr<ui::PangolinWindow>& ui) = 0;
};

}  // namespace lightning::loc
