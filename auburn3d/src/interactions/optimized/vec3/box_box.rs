//! Boxes against boxes have no exact cast yet, so they go through `algorithm::shape_cast`.

use crate::Vec3;
use crate::interactions::optimized::{casts_from_span, support_span};
use crate::shape::box3d::Box3d;

casts_from_span!(Box3d, Vec3, support_span);
