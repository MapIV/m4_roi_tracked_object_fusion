# m4_roi_tracked_object_fusion
Works like Autoware's image_projection_based_fusion::roi_detected_object_fusion except it works with TrackedObjects instead of DetectedObjects. Also we add two parameters:

* allow_no_matching_roi: in case you want objects that fail to get matched to an ROI to not get filtered out, you can set this to true.
* filter_unknown_classes: if you set allow_no_matching_roi to true or you have unknown objects in your camera ROIs, then you can set this to true and UNKNOWN objects after ROI fusion.