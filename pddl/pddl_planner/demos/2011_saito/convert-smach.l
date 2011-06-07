;;
;; Convert from PDDL result (include failure recovery) to State Machine
;;

;; TODO
;; start, goal state
;; for many arc
;; return value

(defun rmnl (str) ;; remove new line
  (map string #'(lambda(c)(if (= c #\\) #\_ c)) str))

(defun convert-smach (gr)
  (let ((sm (instance state-machine :init)) act name sym)
    ;; add nodes
    (dolist (n (send gr :nodes))
      (setq act (send n :neighbor-action-alist)
	    name (rmnl (send n :name)))
      (cond ((null act) ;; no more action
	     (send sm :goal-state (list name)))
	    (t ;; last arc is correct action
	     (setq sym (read-from-string (caar (last act))))
	     (setq sym
		   `(lambda-closure nil 0 0 (x) (apply ',(car sym) ',(cdr sym))))
	     (send sm :add-node (instance state :init name sym)))
	    ))
    ;; add arcs
    (dolist (n (send gr :nodes))
      (setq act (send n :neighbor-action-alist)
	    name (rmnl (send n :name)))
      (cond ((null act))
	    (t
	     (send sm :add-transition name (rmnl (send (cdar (last act)) :name)) t)
	     (when (< 1 (length act))
	       (send sm :add-transition name (rmnl (send (cdar act) :name)) nil)))))

    ;; select a node as start-node
    ;; now this is not always true
    ;; (send sm :start-state (send (send gr :start-state) :name))
    (send sm :start-state (rmnl (send (car (last (send gr :nodes))) :name)))
    sm ))
