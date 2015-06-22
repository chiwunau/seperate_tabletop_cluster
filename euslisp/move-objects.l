(defun sort-boxes-by-dist-from (boxes cds)
  (let ((sorted-boxes (copy-seq boxes)))
    (sort sorted-boxes #'(lambda (b1 b2)
                           (if (<= (abs (norm (v- (send cds :worldpos) (send b1 :worldpos))))
                                   (abs (norm (v- (send cds :worldpos) (send b2 :worldpos)))))
                               t)))
    sorted-boxes))
  

(setq *left-arm-candidates* (sort-boxes-by-dist-from *boxes* (send *pr2* :larm :end-coords)))
(setq *right-arm-candidates* (sort-boxes-by-dist-from *boxes* (send *pr2* :rarm :end-coords)))

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

(defun push-objects ()
  (let ((target-box (car *left-arm-candidates*))
        (other-boxes (cdr *left-arm-candidates*))
        (insert-offset 15))
        ;; target-cds-list)

    (setq target-cds-list (mapcar #'(lambda (cds)
                                      (send cds :translate (float-vector (* -1 insert-offset) 0 0))
                                      (send cds :rotate (deg2rad 90) :y)
                                      (send cds :rotate (deg2rad 180) :z))
                                  (calc-push-coords-list target-box)))

    ;;filter target-cds-list
    (dolist (cds (target-cds-list))
      ;;not moving toward centrod
      (if (< (rad2deg (vector-angle (send cds :z-axis) (v- (send *gba* :boxes-centroid) (send cds :worldpos)))) 90)
          (return))






(defun check-collision (&optional (arm :larm))
  (dolist (link (append (send *pr2* arm :links) (send (send (send *pr2* :larm :end-coords :parent) :parent) :child-links)))
    (dolist (box *boxes*)
      (if (eq (pqp-collision-check link box) 1)
          (return-from check-collision nil))))
  t)
      
          
  




    

    