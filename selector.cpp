#include "selector.h"
#include "camera.h"
#include "controller.h"
#include <iostream>
#include "mesh.h"

using namespace std;

using namespace Eigen;

Selector::Selector(Camera &c) : c_(c), cont_(NULL), state_(SS_NONE), mode_(SM_DRAGFREE)
{
    startPos_.setZero();
}

void Selector::setController(Controller &c)
{
    cont_ = &c;
}

void Selector::startEditing(const Vector2d &pos, SelectMode mode)
{
    state_ = SS_PICKQUEUED;
    mode_ = mode;
    startPos_ = pos;
}

void Selector::primitiveSelected(Mesh::PrimType primtype, int primidx)
{
    dragtype_ = primtype;
    dragidx_ = primidx;
    switch(mode_)
    {
        case SM_DRAGHEIGHT:
        case SM_DRAGFREE:
        {
            state_ = SS_DRAGGING;
            if(primtype == Mesh::PT_VERTEX)
            {
                cont_->setAnchor(primidx);
            }
            break;
        }
        case SM_CLEARANCHOR:
        {
            state_ = SS_NONE;
            if(primtype == Mesh::PT_VERTEX)
            {
                cont_->clearAnchor(primidx);
            }
            break;
        }
        case SM_ADDPIN:
        {
            state_ = SS_NONE;
            if(primtype == Mesh::PT_VERTEX)
            {
                cont_->setPin(primidx);
            }
            break;
        }
        case SM_DELETEPIN:
        {
            state_ = SS_NONE;
            if(primtype == Mesh::PT_VERTEX)
            {
                cont_->clearPin(primidx);
            }
            break;
        }
        case SM_DELETEFACE:
        {
            state_ = SS_NONE;
            if(primtype == Mesh::PT_FACE)
                cont_->deleteFace(primidx);
            break;
        }
        case SM_TOGGLECREASE:
        {
            state_ = SS_NONE;
            if(primtype == Mesh::PT_EDGE)
                cont_->toggleCrease(primidx);
            break;
        }
        default:
            assert(!"Bad select mode");
    }
}

void Selector::updateEditing(const Vector2d &pos)
{
    if(state_ != SS_DRAGGING)
        return;

    Vector2d translation2D = pos-startPos_;
    Vector3d right, up, center;
    c_.getSpanningSet(right, up, center);

    Vector3d translation3D = translation2D[0]*right + translation2D[1]*up;
    if(mode_ == SM_DRAGFREE && dragtype_ == Mesh::PT_VERTEX)
        cont_->dragVertex(dragidx_, translation3D);
    else if(mode_ == SM_DRAGHEIGHT && dragtype_ == Mesh::PT_VERTEX)
        cont_->dragVertexHeight(dragidx_, translation3D);

    startPos_ = pos;
}

void Selector::stopEditing()
{
    state_ = SS_NONE;
}

Vector2d Selector::getQueuedPos()
{
    assert(selectionQueued());
    return startPos_;
}
