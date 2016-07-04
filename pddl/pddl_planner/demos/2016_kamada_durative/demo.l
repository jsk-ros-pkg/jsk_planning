#!/usr/bin/env roseus
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")

(setq *problem* (instance pddl-problem :init :name 'demo-problem :domain 'demo-domain))
(send *problem* :objects
      '((init-pos . position)
        (kitchentable-front . position)
        (trashbox-front . position)
        (fridge-fron . position)
        (kitchentable . object)
        (trashbox . object)
        (robot1 . robot)
        (robot2 . robot)))

(setq *domain* (instance pddl-domain :init :name 'demo-domain :durative-action t))
(send *domain* :requirements '(:typing :durative-actions :adl))
(send *domain* :types '(object position robot))
(send *domain* :constants '(GEORGIA EMPTY-BOX SUSHI-CUP MUG-CUP BOTTLE TABLECLOTH TABLE FRIDGE CART - object
                                    TABLE-FRONT FRIDGE-FRONT CART-FRONT - position))
(send *domain* :predicates '((at-robot ?robot - robot ?robotpos - position)
                             (reachable-from ?robotpos - position ?obj - object)
                             (grasping ?robot - robot ?obj - object)
                             (placing-possible ?obj) ;; avoiding (on CUP GEORGIA) etc
                             (free-arm ?robot - robot)
                             (on ?underobj - object ?obj - object)
                             (in ?fridge - object ?georgia - object)
                             (closed ?fridge - object)
                             (slightly-closed ?fridge - object)
                             (tableclothset)
                             (cangrasped)
                             (blocking ?cart - object)
                             (slightly-blocking ?cart - object)
                             ))

(setq action-lst
      (list
       (instance pddl-action :init
                 :use-durative-action t
                 :name "pick"
                 ;; ?robot pick ?obj from ?underobj at ?robotpos
                 :parameters '((?robot - robot ?obj - object ?underobj - object ?robotpos - position))
                 :duration '((= ?duration 3))
                 :precondition '((over all (at-robot ?robot ?robotpos))
                                 (over all (reachable-from ?robotpos ?underobj))
                                 (over all (placing-possible ?underobj))
                                 (at start (reachable-from ?robotpos ?obj))
                                 (at start (on ?underobj ?obj))
                                 (at start (free-arm ?robot))
                                 (at start (not (grasping ?robot ?obj)))
                                 )
                 :effect '((at start (not (free-arm ?robot)))
                           (at end (not (reachable-from ?robotpos ?obj)))
                           (at end (not (on ?underobj ?obj)))
                           (at end (grasping ?robot ?obj))
                           ))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "place"
                 ;; ?robot place ?obj on ?underobj at ?robotpos
                 :parameters '((?robot - robot ?obj - object ?underobj - object ?robotpos - position))
                 :duration '((= ?duration 3))
                 :precondition '((over all (at-robot ?robot ?robotpos))
                                 (over all (reachable-from ?robotpos ?underobj))
                                 (over all (placing-possible ?underobj))
                                 (at start (not (reachable-from ?robotpos ?obj)))
                                 (at start (not (on ?underobj ?obj)))
                                 (at start (not (free-arm ?robot)))
                                 (at start (grasping ?robot ?obj))
                                 )
                 :effect '((at start (not (grasping ?robot ?obj)))
                           (at end (reachable-from ?robotpos ?obj))
                           (at end (on ?underobj ?obj))
                           (at end (free-arm ?robot))))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "move"
                 ;; ?robot move from ?from to ?to
                 :parameters '((?robot - robot ?from - position ?to - position))
                 :duration '((= ?duration 20))
                 :precondition '((at start (at-robot ?robot ?from))
                                 )
                 :effect '((at start (not (at-robot ?robot ?from)))
                           (at end (at-robot ?robot ?to))))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "set-tablecloth"
                 ;; ?robot set TABLECLOTH on TABLE at TABLE-FRONT
                 :parameters '((?robot - robot))
                 :duration '((= ?duration 18))
                 :precondition '((at start (not (tableclothset)))
                                 (over all (at-robot ?robot TABLE-FRONT))
                                 (over all (reachable-from TABLE-FRONT TABLE))
                                 (over all (not (on TABLE GEORGIA)))
                                 (over all (not (on TABLE EMPTY-BOX)))
                                 (over all (not (on TABLE SUSHI-CUP)))
                                 (over all (not (on TABLE MUG-CUP)))
                                 (over all (not (on TABLE BOTTLE)))
                                 (at start (free-arm ?robot))
                                 (at start (not (on TABLE TABLECLOTH)))
                                 (at start (not (reachable-from TABLE-FRONT TABLECLOTH)))
                                 )
                 :effect '((at start (not (free-arm ?robot)))
                           (at end (free-arm ?robot))
                           (at end (on TABLE TABLECLOTH))
                           (at end (reachable-from TABLE-FRONT TABLECLOTH))
                           (at end (tableclothset))))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "open-fridge"
                 ;; ?robot open FRIDGE at FRIDGE-FRONT
                 :parameters '((?robot - robot))
                 :duration '((= ?duration 20))
                 :precondition '((over all (at-robot ?robot FRIDGE-FRONT))
                                 (over all (reachable-from FRIDGE-FRONT FRIDGE))
                                 (at start (free-arm ?robot))
                                 (at start (closed FRIDGE))
                                 )
                 :effect '((at start (not (free-arm ?robot)))
                           (at end (free-arm ?robot))
                           (at end (not (closed FRIDGE)))))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "grasp-can"
                 ;; ?robot grasp GEORGIA from FRIDGE at FRIDGE-FRONT
                 :parameters '((?robot - robot))
                 :duration '((= ?duration 20))
                 :precondition '((over all (at-robot ?robot FRIDGE-FRONT))
                                 (over all (reachable-from FRIDGE-FRONT FRIDGE))
                                 (over all (reachable-from FRIDGE-FRONT GEORGIA))
                                 (over all (not (closed FRIDGE)))
                                 (at start (in FRIDGE GEORGIA))
                                 (at start (not (cangrasped)))
                                 (at start (free-arm ?robot))
                                 (at start (not (grasping ?robot GEORGIA)))
                                 )
                 :effect '((at start (not (free-arm ?robot)))
                           (at end (not (closed FRIDGE)))
                           (at end (not (in FRIDGE GEORGIA)))
                           (at end (cangrasped))
                           (at end (not (reachable-from FRIDGE-FRONT GEORGIA)))
                           (at end (grasping ?robot GEORGIA))))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "close-fridge"
                 ;; ?robot close FRIDGE at FRIDGE-FRONT
                 :parameters '((?robot - robot))
                 :duration '((= ?duration 20))
                 :precondition '((over all (at-robot ?robot FRIDGE-FRONT))
                                 (over all (reachable-from FRIDGE-FRONT FRIDGE))
                                 (at start (cangrasped))
                                 (at start (not (closed FRIDGE)))
                                 )
                 :effect '((at start (slightly-closed FRIDGE))
                           (at end (not (slightly-closed FRIDGE)))
                           (at end (closed FRIDGE))))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "remove-cart"
                 ;; ?robot remove CART at CART-FRONT
                 :parameters '((?robot - robot))
                 :duration '((= ?duration 20))
                 :precondition '((over all (at-robot ?robot CART-FRONT))
                                 (over all (reachable-from CART-FRONT CART))
                                 (at start (blocking CART))
                                 )
                 :effect '((at start (slightly-blocking CART))
                           (at end (not (slightly-blocking CART)))
                           (at end (not (blocking CART)))))
       ))
(dolist (ac action-lst)
  (send *domain* :add :action ac))

(send *problem* :initial-condition
      '((at-robot robot1 init-pos)
        (at-robot robot2 init-pos)
        (reachable-from TABLE-FRONT TABLE)
        (reachable-from TABLE-FRONT EMPTY-BOX)
        (reachable-from TABLE-FRONT SUSHI-CUP)
        (reachable-from TABLE-FRONT MUG-CUP)
        (reachable-from TABLE-FRONT BOTTLE)
        (reachable-from FRIDGE-FRONT FRIDGE)
        (reachable-from FRIDGE-FRONT GEORGIA)
        (reachable-from CART-FRONT CART)
        (reachable-from kitchentable-front kitchentable)
        (reachable-from trashbox-front trashbox)
        (not (reachable-from TABLE-FRONT TABLECLOTH))
        (placing-possible TABLE)
        (placing-possible TABLECLOTH)
        (placing-possible kitchentable)
        (placing-possible trashbox)
        (free-arm robot1)
        (free-arm robot2)
        (not (busy robot1))
        (not (busy robot2))
        (on TABLE EMPTY-BOX)
        (on TABLE SUSHI-CUP)
        (on TABLE MUG-CUP)
        (on TABLE BOTTLE)
        (closed FRIDGE)
        (in FRIDGE GEORGIA)
        (blocking CART)
        (not (tableclothset))))

(send *problem* :goal-condition
      '(
        (reachable-from TABLE-FRONT TABLE)
        (reachable-from TABLE-FRONT GEORGIA)
        (reachable-from kitchentable-front SUSHI-CUP)
        (reachable-from kitchentable-front MUG-CUP)
        (reachable-from kitchentable-front BOTTLE)
        (reachable-from kitchentable-front kitchentable)
        (reachable-from FRIDGE-FRONT FRIDGE)
        (reachable-from TABLE-FRONT TABLECLOTH)
        (reachable-from trashbox-front EMPTY-BOX)
        (on kitchentable SUSHI-CUP)
        (on kitchentable MUG-CUP)
        (on kitchentable BOTTLE)
        (on trashbox EMPTY-BOX)
        (on TABLECLOTH GEORGIA)
        (on TABLE TABLECLOTH)
        (closed FRIDGE)
        (not (blocking CART))
        (tableclothset)
        ))

(defun demo ()
  (setq *result* (solve-pddl-planning *domain* *problem*))
  (cond
   ((equal (send (cdr (assoc :state *result*)) :generate-relationship) 'error)
    (ros::ros-error "LPG planner error!"))
   (t
    (setq *graph* (make-simple-graph *result*))
    (write-to-dot *graph*)
    (unix:system (format nil "dot ~A -Tpdf -o ~A" "actionrelation.dot" "actionrelation.pdf"))
    (piped-fork "gnome-open actionrelation.pdf"))
    ))

(defun make-simple-graph (result) ;; not use start and end.
  (let ((relation-lst (send (cdr (assoc :state result)) :relationship))
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

(defun write-to-dot (gr)
  (let ((node-alist
         (mapcar #'(lambda (n)
                     (cons n (string (gensym))))
                 (send gr :nodes))))
    (labels ((graphviz-node-name
              (n)
              (cdr (assoc n node-alist))))
      (with-open-file
       (f "actionrelation.dot" :direction :output)
       (format f "digraph ~A {~%" "actionrelation")
       (dolist (nd (send gr :nodes))
         (format f "   ")
         (format f "~A [label = \"~A\"];~%"
                 (graphviz-node-name nd)
                 (send nd :name)))
       (let ((drawed-arc nil))
         (dolist (target-node (send gr :nodes))
           (let ((neighbors (send target-node :neighbor-action-alist)))
             (dolist (neighbor neighbors)
               (format f "   ")
               (format f "~A -> ~A [label = \"~A\"];~%"
                       (graphviz-node-name target-node)
                       (graphviz-node-name (cdr neighbor)) (car neighbor))
               )))
         (format f "}~%")
         t)))))




(demo)
