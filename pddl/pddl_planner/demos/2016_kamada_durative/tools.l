

(defun make-relation-graph (result &key verbose)
  (let ((relation-lst
         (cond
          (verbose
           (send (cdr (assoc :state result)) :all-relationship))
          (t
           (send (cdr (assoc :state result)) :relationship))))
        (initial-node (instance arced-node :init :name (format nil "INITIAL")))
        (start-actions
         (remove-if #'null
                      (mapcar #'(lambda (x)
                                  (when (member 'start x) x))
                              (assoc :plan result))))
        (gr (instance pddl-graph :init)))
    (ros::ros-info "[length] relation-lst:~A start-actions:~A" (length relation-lst) (length start-actions))
    (dolist (sac start-actions)
      (unless (find-if-not #'null
                           (mapcar #'(lambda (x)
                                       (cond ((equal (send x :name) (format nil "~A ~A" (car sac) (caddr sac))) x)
                                             (t nil)))
                                   (send gr :nodes)))
        (send gr :add-node
              (instance arced-node :init :name
                        (format nil "~A ~A" (car sac) (caddr sac))))))
    ;; add INITIAL
    (let (first-node)
      (send gr :add-node initial-node)
      (setq first-node
            (find-if-not #'null
                         (mapcar #'(lambda (x)
                                     (cond ((equal (send x :name) (format nil "~A ~A"
                                                                          (car (car start-actions))
                                                                          (caddr (car start-actions)))) x)
                                           (t nil)))
                                 (send gr :nodes))))
      (send gr :add-arc-from-to initial-node first-node (format nil "")))
    (dolist (rl relation-lst)
      (let (from-node to-node)
        (setq to-node
              (find-if-not #'null
                           (mapcar #'(lambda (x)
                                       (cond ((equal (send x :name) (format nil "~A ~A"
                                                                            (car (car rl))
                                                                            (caddr (car rl)))) x)
                                             (t nil)))
                                   (send gr :nodes))))
        (let ((reverse-lst (member to-node (send gr :nodes))))
          (setq from-node
                (find-if-not #'null
                             (mapcar #'(lambda (x)
                                         (cond ((substringp (format nil "~A" (caddr (cadr rl))) (send x :name)) x)
                                                         (t nil)))
                                     reverse-lst)))
          )
        (when (and from-node to-node)
          (send gr :add-arc-from-to from-node to-node (format nil "~A" (cddr rl))))
        ))
    ;; link start nodes to initial-node
    (let ((tmp-nodes (send gr :nodes)))
      (dolist (nd (send gr :nodes))
        (dolist (aarc (send nd :arc-list))
          (setq tmp-nodes (remove (send aarc :to) tmp-nodes :count 1))))
      (setq tmp-nodes (remove initial-node tmp-nodes :count 1))
      (dolist (tn tmp-nodes)
        (send gr :add-arc-from-to initial-node tn (format nil ""))))
    gr
    ))

(defun write-to-dot (gr filename &key distinguish-robot)
  (let ((node-alist
         (mapcar #'(lambda (n)
                     (cons n (string (gensym))))
                 (send gr :nodes)))
        robot1-node-lst
        robot2-node-lst
        )
    (labels ((graphviz-node-name
              (n)
              (cdr (assoc n node-alist))))
      (with-open-file
       (f filename :direction :output)
       (format f "digraph ~A {~%" "actionrelation")
       (when distinguish-robot
         (format f "subgraph cluster_0 {~%label = \"robot 1\";~%color=blue;~%")
         (dolist (nd (send gr :nodes))
           (when (substringp "robot1" (send nd :name))
             (format f "~A;~%" (graphviz-node-name nd))
             (setq robot1-node-lst (append robot1-node-lst (list (graphviz-node-name nd))))
             ))
         (format f "}~%")

         (format f "subgraph cluster_1 {~%label = \"robot 2\";~%color=blue;~%")
         (dolist (nd (send gr :nodes))
           (when (substringp "robot2" (send nd :name))
             (format f "~A;~%" (graphviz-node-name nd))
             (setq robot2-node-lst (append robot2-node-lst (list (graphviz-node-name nd))))
             ))
         (format f "}~%")
         )
       (dolist (nd (send gr :nodes))
         (format f "   ")
         (format f "~A [label = \"~A\"];~%"
                 (graphviz-node-name nd)
                 (send nd :name)))
       (let ((drawed-arc nil))
         (dolist (target-node (send gr :nodes))
           (let ((neighbors (send target-node :neighbor-action-alist)))
             (dolist (neighbor neighbors)
               (when
                   (or (not (eq distinguish-robot :inter-robot-only))
                       (not
                        (or
                         (subsetp (list (graphviz-node-name target-node)
                                        (graphviz-node-name (cdr neighbor)))
                                  robot1-node-lst)
                         (subsetp (list (graphviz-node-name target-node)
                                        (graphviz-node-name (cdr neighbor)))
                                  robot2-node-lst))))
                 (format f "   ")
                 (format f "~A -> ~A [label = \"~A\"];~%"
                         (graphviz-node-name target-node)
                         (graphviz-node-name (cdr neighbor)) (car neighbor))
                 ))))
         (when (eq distinguish-robot :inter-robot-only)
           (let (from-node to-node)
             (dolist (nlst robot1-node-lst)
               (setq from-node nlst)
               (when to-node
                 (format f "~A -> ~A;~%" from-node to-node))
               (setq to-node from-node)
               ))
           (let (from-node to-node)
             (dolist (nlst robot2-node-lst)
               (setq from-node nlst)
               (when to-node
                 (format f "~A -> ~A;~%" from-node to-node))
               (setq to-node from-node)
               )))
         (format f "}~%")
         t)))))

(defun demo ()
  (setq *result* (solve-pddl-planning *domain* *problem*))
  (cond
   ((equal (send (cdr (assoc :state *result*)) :generate-single-relationship) 'error)
    (ros::ros-error "LPG planner error!"))
   (t
    (send (cdr (assoc :state *result*)) :generate-all-relationship)
    (setq *graph* (make-relation-graph *result*))
    (setq *all-graph* (make-relation-graph *result* :verbose t))
    (write-to-dot *graph* ".normal.dot")
    (write-to-dot *graph* ".normal_separated.dot" :distinguish-robot t)
    (write-to-dot *all-graph* ".verbose.dot")
    (write-to-dot *all-graph* ".verbose_separated.dot" :distinguish-robot t)
    (write-to-dot *all-graph* ".inter_robot_relationship.dot" :distinguish-robot :inter-robot-only)
    (unix:system (format nil "dot ~A -Tpdf -o ~A" ".normal.dot" ".normal.pdf"))
    (unix:system (format nil "dot ~A -Tpdf -o ~A" ".normal_separated.dot" ".normal_separated.pdf"))
    (unix:system (format nil "dot ~A -Tpdf -o ~A" ".verbose.dot" ".verbose.pdf"))
    (unix:system (format nil "dot ~A -Tpdf -o ~A" ".verbose_separated.dot" ".verbose_separated.pdf"))
    (unix:system (format nil "dot ~A -Tpdf -o ~A" ".inter_robot_relationship.dot" ".inter_robot_relationship.pdf"))
    (piped-fork "gnome-open .normal.pdf")
    (piped-fork "gnome-open .normal_separated.pdf")
    (piped-fork "gnome-open .verbose.pdf")
    (piped-fork "gnome-open .verbose_separated.pdf")
    (piped-fork "gnome-open .inter_robot_relationship.pdf")
    ))
  )
