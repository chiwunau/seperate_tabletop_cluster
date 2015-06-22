(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::roseus "generate-tabletop-scene")

(setq *polygon-array-topic* "/multi_plane_estimate/output_refined_polygon")
(setq *boundingbox-array-topic* "/cluster_decomposer_final/boxes")
(setq *table* nil)
(setq *boxes* nil)


(defclass generate-polygon-array
  :slots 
  (*vertices-list-org* *plane-normal-list* *latest-polygon-list* *polygon-list* *cloud-stamp* *cloud-centroid-list* *plgs-cb-flag* *cofs-cb-flag* *tfl*))

(defmethod generate-polygon-array
  (:init 
   ()
   (ros::subscribe *polygon-array-topic* jsk_recognition_msgs::PolygonArray #'send self :polygon-array-cb)
   (setq *tfl* (instance ros::transform-listener :init)))

  (:polygon-array-cb
   (msg)
   (ros::ros-info "polygons callback")
   (setq *plgs-cb-flag* t)
   (setq *cloud-stamp* (send msg :header :stamp))
   (let ((plg-list (send msg :polygons)))

     (setq *vertices-list-org* (mapcar #'(lambda (plg)
                                           (let ((pts (send (send plg :polygon) :points))
                                                 pts-list
                                                 trans
                                                 trans-pt)
                                             (setq trans (send *tfl* :lookup-transform "/base_footprint" "/openni_rgb_optical_frame" (ros::time 0)))
                                             (dolist (pt pts)
                                               (setq trans-pt (send trans :transform-vector
                                                                    (scale 1000 (float-vector (send pt :x) (send pt :y) (send pt :z)))))
                                               (push trans-pt pts-list))
                                             (reverse pts-list)))
                                       plg-list))
     ;; (dotimes (i (length *vertices-list-org*))
     ;;   (let* ((c (send *tfl* :lookup-transform "/base_footprint" (format nil "/multi_plane_centroidoutput~0,2D" i) *cloud-stamp*)))
     ;;     (when c
     ;;       (send c :transform (send *pr2* :copy-worldcoords) :world)
     ;;       (push  (send c :copy-worldcoords) centroid-list))))
     ;; (setq *cloud-centroid-list* (reverse centroid-list))
     (send self :init-eus-polygons)))
  
  (:init-eus-polygons
   ()
   ;; (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   ;; (send *irtviewer* :objects (list *pr2*))
   (let (polygons-list)
     ;;shift the detected polygons a little considering the thickness
     (dotimes (i (length *vertices-list-org*))
	(let ((plg-temp (instance polygon :init :vertices (elt *vertices-list-org* i)))
	      plg v)

	  ;; (if (>= (v. (matrix-column (send *pr2* :head :end-coords :worldrot) 2) (send plg-temp :normal)) 0) ;;shift for table thickness
	  ;;     (setq v (send plg-temp :normal))
	  ;;   (setq v (scale -1.0 (send plg-temp :normal))))
	  (setq plg (instance polygon :init :vertices (send plg-temp :vertices)))
	  ;; (setq plg (instance polygon :init :vertices (mapcar #'(lambda (pt) (v+ pt (scale 25 v))) (send plg-temp :vertices))))
	  (push plg polygons-list)))

      (setq *latest-polygon-list* (reverse polygons-list))))

  (:draw-polygons
   (plgs)
   (mapcar #'(lambda (plg ct)
               (send plg :reset-normal)
               (let* (;;(color (float-vector (random 1.0) (random 1.0) (random 1.0)))
                      (color #f(0.87 0.72 0.53)) ;;burlywood color
                      (prism (make-prism (mapcar #'(lambda (p) (scale 1 p)) (send plg :vertices)) (send plg :normal)))
                      (sphere (make-sphere 20)))
                 (send sphere :move-to ct :world)
                 (send sphere :set-color :white)
                 (send prism :set-color color)
                 ;; (send *irtviewer* :objects sphere)
                 (send *irtviewer* :objects prism)))
           plgs
           ;; (if (equal (length *cloud-centroid-list*) (length polygons-list))
           ;;     (progn (print "check1")
           ;;            *cloud-centroid-list*)
           ;;   (progn (print "check2")
           (mapcar #'(lambda (p) (make-coords :pos (elt (send p :centroid) 1))) plgs))
   (send *irtviewer* :draw-objects :flush t)
   )
  
  (:sort-polygons-by-dist-from-head
   (&optional (plgs *polygon-list*) (pt (send (car (send *pr2* :head)) :worldpos)))
   (sort plgs #'(lambda (plg1 plg2)
                  (if (<= (norm (v- (elt (send plg1 :centroid) 1) pt)) (norm (v- (elt (send plg2 :centroid) 1) pt)))
                      t))))
   

  (:update-polygons
   ()
   (setq *polygon-list* (copy-object *latest-polygon-list*)))

  (:get-polygons
   ()
   *polygon-list*)

  )

(defclass generate-boundingbox-array
  :slots
  (*latest-box-list* *box-list* *boxs-stamp*))

(defmethod generate-boundingbox-array
  (:init
   ()
   (ros::subscribe *boundingbox-array-topic* jsk_recognition_msgs::BoundingBoxArray #'send self :boundingbox-array-cb))

  (:boundingbox-array-cb
   (msg)
   (ros::ros-info "boundingbox callback")
   (setq *boxes-stamp* (send msg :header :stamp))

   (let ((boxes (send msg :boxes))
         (trans (send *tfl* :lookup-transform "/base_footprint" "/openni_rgb_optical_frame" (ros::time 0))))
     (setq *latest-box-list*
           (mapcar #'(lambda (box)
                       (let* ((dim (ros::tf-point->pos (send box :dimensions)))
                              (cube (make-cube (elt dim 0) (elt dim 1) (elt dim 2)))
                              (cds (ros::tf-pose->coords (send box :pose))))
                         (send cds :transform trans :world)
                         (send cube :move-to cds)
                         (send cube :set-color (float-vector (random 1.0) (random 1.0) (random 1.0)))
                         cube))
                   boxes))))

  (:draw-boxes
   (boxes)
   (send *irtviewer* :objects (append (send *irtviewer* :objects) boxes))
   (send *irtviewer* :draw-objects :flush t))

  (:update-boxes
   ()
   (setq *box-list* (copy-object *latest-box-list*)))

  (:get-boxes
   ()
   *box-list*)

  (:filter-boxes
   ()
   (let (boxes-temp)
     (dolist (b *box-list*)
       (if (> (elt (send b :worldpos) 2) (elt (cadr (send *table* :centroid)) 2))
           (push b boxes-temp)))
     (setq *box-list* (reverse boxes-temp))))

  (:boxes-centroid
   ()
   (let ((sum #f(0 0 0)))
     (dolist (b *box-list*)
       (setq sum (v+ sum (send b :worldpos))))
     (setq sum (scale (/ 1.0 (length *box-list*)) sum))
     sum))
  )


(defun init-scene ()
  (setq *gpa* (instance generate-polygon-array :init))
  (setq *gba* (instance generate-boundingbox-array :init)))

(defun update-scene ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *irtviewer* :objects (list *pr2*))
  (setq *table* nil)
  (setq *boxes* nil)
  (while t
    (ros::spin-once)
    (if (and (send *gpa* :update-polygons) (send *gba* :update-boxes))
        (return)
      (ros::spin-once)))
  (send *gpa* :sort-polygons-by-dist-from-head)
  (setq *table* (car (send *gpa* :get-polygons)))
  (send *gba* :filter-boxes)
  (setq *boxes* (send *gba* :get-boxes))

  (send *gpa* :draw-polygons (list *table*))
  (send *gba* :draw-boxes *boxes*))


