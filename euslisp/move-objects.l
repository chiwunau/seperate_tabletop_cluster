(defun sort-boxes-by-dist-from (boxes cds)
  (let ((sorted-boxes (copy-seq boxes)))
    (sort sorted-boxes #'(lambda (b1 b2)
                           (if (<= (abs (norm (v- (send cds :worldpos) (send b1 :worldpos))))
                                   (abs (norm (v- (send cds :worldpos) (send b2 :worldpos)))))
                               t)))
    sorted-boxes))
  

(defun calc-push-coords-list (box)
  (let (cds-list
        cds-p cds-n)
    ;;x direction
    (setq cds-p (send box :copy-worldcoords))
    (setq cds-n (send box :copy-worldcoords))
    (send cds-p :translate (float-vector (/ (elt (car (send box :csg)) 1) 2) 0 0))
    (send cds-n :translate (float-vector (/ (* -1 (elt (car (send box :csg)) 1)) 2) 0 0))
    (send cds-p :rotate (deg2rad 180) :z)
    (push cds-p cds-list)
    (push cds-n cds-list)
    ;; y direction
    (setq cds-p (send box :copy-worldcoords))
    (setq cds-n (send box :copy-worldcoords))
    (send cds-p :translate (float-vector 0 (/ (elt (car (send box :csg)) 2) 2) 0))
    (send cds-n :translate (float-vector 0 (/ (* -1 (elt (car (send box :csg)) 2)) 2) 0))
    (if (>= (abs (rad2deg (vector-angle (send box :z-axis) #f(0 0 1)))) 90)
        (progn
          (send cds-p :rotate (deg2rad -90) :z)
          (send cds-n :rotate (deg2rad 90) :z))
      (progn
        (send cds-p :rotate (deg2rad 90) :z)
        (send cds-n :rotate (deg2rad -90) :z)))
    (push cds-p cds-list)
    (push cds-n cds-list)
    cds-list))

(defun check-collision (&optional (arm :larm))
  (dolist (link (append (send *pr2* arm :links) (send (send (send *pr2* :larm :end-coords :parent) :parent) :child-links)))
    (dolist (box *boxes*)
      (if (eq (pqp-collision-check link box) 1)
          (return-from check-collision nil))))
  t)


(defun filter-pushing-cds (target-box other-boxes &optional (arm :larm))
  (let ((insert-offset 30)
        target-cds-list
        filter-target-cds-list)

    (setq target-cds-list (mapcar #'(lambda (cds)
                                      (send cds :translate (float-vector (* -1 insert-offset) 0 0))
                                      (send cds :rotate (deg2rad 90) :y)
                                      (send cds :rotate (deg2rad 180) :z))
                                  (calc-push-coords-list target-box)))

    ;;filter target-cds-list
    (dolist (cds target-cds-list)
      (print cds)
      (block :go-next-cds
        (let ((cds-rotate? (list nil 0)))
          ;;not moving toward centrod
          (if (< (rad2deg (vector-angle (send cds :z-axis) (v- (send *gba* :boxes-centroid) (send cds :worldpos)))) 80)
              (return-from :go-next-cds))
          ;;arm will not collide
          (send *pr2* :inverse-kinematics cds :move-target (send *pr2* :larm :end-coords) :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent)) :use-torso t :debug-view nil)
          (if (not (check-collision :larm))
              (progn
                (send *pr2* :inverse-kinematics (send (send cds :copy-worldcoords) :rotate (deg2rad 180) :x) :move-target (send *pr2* :larm :end-coords) :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent)) :use-torso t)
                (if (not (check-collision :larm))
                    (return-from :go-next-cds)
                  (setf (elt cds-rotate? 1) 1))))
          (print "check2")
          ;;no obstacle on path
          (let ((tmp-box (copy-object target-box)))
            (send tmp-box :set-color :yellow)
            (send target-box :set-color :red)
            (send tmp-box :translate #f(0 0 50) cds)
            (objects tmp-box)
            (dolist (ob other-boxes)
              (if (eq (pqp-collision-check tmp-box ob) 1)
                  (progn
                    (send *irtviewer* :objects (delete tmp-box (send *irtviewer* :objects) :count 1))
                    (return-from :go-next-cds))))
            (send tmp-box :translate #f(0 0 50) cds)
            (send *irtviewer* :draw-objects :flush t)
            (send *irtviewer* :objects (delete tmp-box (send *irtviewer* :objects) :count 1))
            (dolist (ob other-boxes)
              (if (eq (pqp-collision-check tmp-box ob) 1)
                  (return-from :go-next-cds))))
          (print "check1")
          (setf (elt cds-rotate? 0) cds)
          (push cds-rotate? filter-target-cds-list))))
    filter-target-cds-list))

(defun seperate-scene ()
  (dolist (target-box *left-arm-candidates*)
    (let ((other-boxes (remove target-box *left-arm-candidates*)))
        (push-boxes target-box other-boxes :larm))))
            
      

(defun push-boxes (target-box other-boxes &optional (arm :larm))
  (let (final-cds-rotate?-list)
    ;;     target-box
    ;;     other-boxes)
    ;; (setq target-box (car *left-arm-candidates*))
    ;; (setq other-boxes (cdr *left-arm-candidates*))
    (setq final-cds-rotate?-list (filter-pushing-cds target-box other-boxes :larm))

    (when final-cds-rotate?-list
      (let* ((obj-cds (make-cascoords :coords (send (car (car final-cds-rotate?-list)) :copy-worldcoords)))
             (tgt-cds (make-cascoords :coords (send obj-cds :copy-worldcoords)))
             (rotate-gripper (cadr (car final-cds-rotate?-list))))
        (if rotate-gripper (send tgt-cds :rotate (deg2rad 180) :x))
        (send obj-cds :assoc tgt-cds)

        (unless (send *pr2* :inverse-kinematics (send (send tgt-cds :copy-worldcoords) :translate #f(-100 0 0)) ;;prepare cds (higher than tgt-cds)
                      :move-target (send *pr2* :larm :end-coords) :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent)) :use-torso t :debug-view nil)
          (unless (send *pr2* :inverse-kinematics (send (send tgt-cds :copy-worldcoords) :translate #f(-50 0 0)) ;;prepare cds (higher than tgt-cds)
                        :move-target (send *pr2* :larm :end-coords) :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent)) :use-torso t :debug-view nil) (return))
        (send *ri* :angle-vector (send *pr2* :angle-vector) 8000)
        (print "move-to point 0")
        (send *ri* :wait-interpolation)

        (unless (send *pr2* :inverse-kinematics tgt-cds ;;prepare cds (higher than tgt-cds)
              :move-target (send *pr2* :larm :end-coords) :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent)) :use-torso t :debug-view nil) (return))
        (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
        (print "move-to point 1")
        (send *ri* :wait-interpolation)

        (send obj-cds :translate #f(0 0 100))
        (unless (send *pr2* :inverse-kinematics tgt-cds ;;prepare cds (higher than tgt-cds)
                      :move-target (send *pr2* :larm :end-coords) :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent)) :use-torso t :debug-view nil) (return))
        (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
        (print "move-to point 3")
        (send *ri* :wait-interpolation))))))






        
        
                    
       
         
        
  


  
      
          
  




    

    