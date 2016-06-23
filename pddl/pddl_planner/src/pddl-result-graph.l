(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")

(defclass pddl-graph
  :super graph
  :slots (cntr))

(defmethod pddl-graph
  (:init ()
   (setq cntr 0)
   (send-super :init))
  (:search-node-from-pddl-state (state)
   (find-if-not #'(lambda (x) (set-exclusive-or state (get x :pddl-state) :test #'equal)) nodes))
  (:add-pddl-state-node
   (pddl-state &key (node-name))
   (let ((nd (send self :search-node-from-pddl-state pddl-state)))
     (unless nd
       (let ((n (instance arced-node :init :name
                          (cond
                           ((eq node-name :simple)
                            (intern (string-upcase (format nil "node_~A" cntr))))
                           ((eq node-name :pprint)
                            (let ((str (make-string-output-stream 128)))
                              (cond
                               ((/= (length pddl-state) 1)
                                (format str "(")
                                (format str "~A\\n" (car pddl-state))
                                (dolist (e (butlast (cdr pddl-state)))
                                  (format str "~A\\n" e))
                                (format str "~A)" (car (last pddl-state))))
                               (t ;; length = 1
                                (format str "~A" pddl-state)
                                ))
                              (get-output-stream-string str)))
                           (t (string-upcase (format nil "~A" pddl-state)))))))
         (incf cntr)
         (setf (get n :pddl-state) pddl-state)
         (send self :add-node n)
         (setq nd n)))
     nd))
  (:add-arc-from-to ;; redefine
   (from to name &key (both nil))
   (if (find-if #'(lambda (x) (and (eq (cdr x) to)
                                   (string-equal (car x) name)))
                (send from :neighbor-action-alist))
       (warn ";; same arc found ~A~%" name)
     (send from :add-neighbor to name)))
  (:write-to-dot ;; redefine for adding arc name
   (fname &optional result-path (title "output"))
    (let ((node-alist  ; ((node . symbol) (node . symbol) ...)
           (mapcar #'(lambda (n)
                       (cons n (string (gensym))))
                   nodes)))
      (labels ((graphviz-node-name (n)
                (cdr (assoc n node-alist))))
      (with-open-file (f fname :direction :output)
        (format f "digraph ~A {~%" title)
        (dolist (target-node nodes)
          (format f "   ")
          (format f "~A [label = \"~A\"];~%"
                  (graphviz-node-name target-node)
                  (send target-node :name)))

        (let ((drawed-arc nil))
          #|  ;; first of all, write result-path
          (let (anode)
            (while (and (setq anode (pop result-path)) result-path)
              (format f "   ")
              (format f "~A -> ~A [color = red];~%"
                      (graphviz-node-name (send anode :state))
                      (graphviz-node-name (send (car result-path) :state)))))
          |#
          (dolist (target-node nodes)
            (let ((neighbors (send target-node :neighbor-action-alist)))
              (dolist (neighbor neighbors)
                (format f "   ")
                (format f "~A -> ~A [label = \"~A\"];~%"
                        (graphviz-node-name target-node)
                        (graphviz-node-name (cdr neighbor)) (car neighbor))
                )))
          (format f "}~%")
          t)))))
  )

(defun make-graph-from-pddl-results (results &key ((:graph gr) (instance pddl-graph :init)) (node-name))
  (if (symbolp (caar results)) (setq results (list results)))

  (let (node-pool (cntr 0))
    (dolist (res results)
      ;; sort
      (dolist (l (cdr (assoc :step-state res)))
        (sort-condition l))

      (let (nd ns (plan (cdr (assoc :plan res))))
        (when (assoc :durative res)
          (let (tmp-plan)
            (dolist (pp plan)
              (setq tmp-plan (append tmp-plan (list (format nil "~A" (append (list (read-from-string (format nil "~A-~A" (caaddr pp) (cadr pp)))) (cdaddr pp))))))
              (setq plan tmp-plan)
              )))
        (dolist (l (cdr (assoc :step-state res)))
          ;; add node
          (setq nd (send gr :add-pddl-state-node l :node-name node-name))
          ;; add arc
          (when (and nd ns)
            (send gr :add-arc-from-to ns nd (format nil "~A" (pop plan))))
          (setq ns nd nd nil)
          )))
    gr))

(defun append-failed-result (result act &key (act-fail) (additional-conditions) ((:graph gr)) (node-name))
  (let ((ret (if act-fail
                 (make-failed-condition result act :act-fail act-fail)
               (make-failed-condition result act)))
        res lst
        (problem (copy-object *problem*)))

    (dolist (r ret)
      (send problem :initial-condition (cdr (assoc :initial-condition r)))

      (setq res (solve-pddl-planning *domain* problem :optimize-state nil))

      ;; do not add the condition already exists
      (when (not (send gr :search-node-from-pddl-state (cadr (assoc :step-state res))))
        (push res lst)
        (setq gr (make-graph-from-pddl-results (list res) :graph gr :node-name node-name)))
      #|(when res
        (push res lst)
        (setq gr (make-graph-from-pddl-results (list res) :graph gr :node-name node-name)))|#
      (send gr :add-arc-from-to
            (send gr :search-node-from-pddl-state (cdr (assoc :src-condition r)))
            (send gr :search-node-from-pddl-state (cadr (assoc :step-state res)))
            (format nil "~A" (cdr (assoc :act-fail r))))
      )
    lst))

(defun add-failed-nodes (result failed-action-list &key ((:graph gr) (instance pddl-graph :init)))
  (let ((graph (make-graph-from-pddl-results (list result) :node-name :pprint :graph gr)) ;; original plan
        (results (list result)))
    (let ((count 0) (count2 -1))
      (while (/= count count2)
        (let (lst)
          (dolist (failed-action failed-action-list)
            (dolist (r results)
              (let ((ret (if (atom failed-action)
                             (append-failed-result r failed-action :graph graph :node-name :pprint)
                           (append-failed-result r (car failed-action) :act-fail (cdr failed-action)
                                                 :graph graph :node-name :pprint)
                           )))
                (if ret (setq lst (append lst ret))))))
          ;;(setq results (append lst results)) ;; ??
          (setq results lst))
        (setq count2 count
              count (apply #'+ (mapcar #'(lambda(x)(length (send x :arc-list))) (send graph :nodes))))
        ))
    ;; results
    graph))

;; remove static condition from graph
(defun make-readable-graph (gr &key (copy t) (remove-not t))
  (let ((grs (if copy (copy-object gr) gr))
        size sexps dups)
    (setq size (length (send grs :nodes)))
    (setq sexps (mapcan
                 #'(lambda(x)(remove '|n| (read-from-string x)))
                 (send-all (send grs :nodes) :name)))
    (while sexps
      (when (= size (count (car sexps) sexps :test #'equal))
        (push (car sexps) dups))
      (setq sexps (remove (car sexps) sexps :test #'equal)))
    (dolist (node (send grs :nodes))
      (let (tmp)
        (setq tmp (remove '|n| (read-from-string (send node :name))))
        (dolist (s dups) (setq tmp (remove s tmp :test #'equal)))
        (when remove-not
          (setq tmp (remove-if
                     #'(lambda (stat) (substringp "NOT-" (symbol-string (car stat))))
                     tmp)))
        (let ((str (make-string-output-stream 128)))
          (cond
           ((/= (length tmp) 1)
            (format str "(")
            (format str "~A\\n" (car tmp))
            (dolist (e (butlast (cdr tmp)))
              (format str "~A\\n" e))
            (format str "~A)" (car (last tmp))))
           (t ;; length = 1
            (format str "~A" tmp)
            ))
          (send node :name (get-output-stream-string str)))
        ))
    grs
    ))

(defun pddl-plan-to-graph (goal-condition
                           &key (domain *domain*) (problem *problem*)
                           (failed-nodes) (readable t) (debug))
  (let ((now-problem (copy-object problem))
        result ret)
    (if goal-condition (send now-problem :goal-condition goal-condition))

    (setq result (solve-pddl-planning domain now-problem))
    (setq ret (make-graph-from-pddl-results (list result) :node-name :pprint))
    (if failed-nodes
        (setq ret (add-failed-nodes result failed-nodes)))
    (if readable (make-readable-graph ret :copy nil))
    (when debug
      (setq *result* result)
      (setq *graph* ret)
      (send ret :write-to-pdf "debug.pdf"))
    ret))

(defun sort-condition (lst &key (ignore-not t))
  (sys::gc) ;; it's darty hack. we should rewrite in eus/c/sequence.c or not use key
  (sort lst #'string<=
        (if ignore-not
            #'(lambda(x)
                (let ((y (format nil "~A" x)))
                  (cond ((and (listp x) (eq 'not (car x))) ;;(not (hoge))
                         (format nil "~A" (cadr x)))
                        ((string= "not-" (subseq y 1 5)) ;; (not-hoge)
                         (format nil "(~A" (subseq y 5)))
                        (t y))))
          #'(lambda(x)(format nil "~A" x)))))

(defun make-failed-condition (result act &key (act-fail (intern (string-upcase (format nil "~A_f" (string act))))))
  (let ((plan (cdr (assoc :plan result)))
        (step-state (cdr (assoc :step-state result)))
        (state (cdr (assoc :state result)))
        pcond ret)
    (while plan
      (let ((act-pos (position-if #'(lambda (x) (equal (car x) act)) plan)))
        (if act-pos
            (setq pcond (cons (elt plan act-pos) (elt step-state act-pos)))
          (setq pcond nil))
        (setq plan (cdr (member (car pcond) plan :test #'equal)))
        (setq step-state (cdr (member (cdr pcond) step-state :test #'equal)))
        (when pcond
          (let ((swap-act (cons act-fail (cdar pcond)))
                icond)
            (send state :initial-state (cdr pcond))
            (send state :actions (list swap-act))
            (setq icond (cadr (send state :generate-states)))
            (if icond
                (push
                 (list
                  (cons :initial-condition icond)
                  (cons :act (car pcond))
                  (cons :act-fail swap-act)
                  (cons :src-condition (sort-condition (cdr pcond)))
                  (cons :dst-condition (sort-condition icond)))
                 ret))))))
    (nreverse ret)))

(provide :pddl-result-graph)
