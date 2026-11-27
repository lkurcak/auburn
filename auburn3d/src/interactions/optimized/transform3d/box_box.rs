//! Boxes against boxes under a [`Transform3d`] go through `algorithm::shape_cast`.

use crate::interactions::optimized::{casts_from_span, support_span};
use crate::shape::box3d::Box3d;
use crate::transformation::Transform3d;

casts_from_span!(Box3d, Transform3d, support_span);
