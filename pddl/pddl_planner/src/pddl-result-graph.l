;; require jskeus
#-:jsk (jsk)

(defclass pddl-graph
  :super graph
  :slots (cntr))

(defmethod pddl-graph
  (:init ()
   (setq cntr 0)
   (send-super :init))
  (:search-node-from-pddl-state (state)
   (find-if #'(lambda (x) (equal state (get x :pddl-state))) nodes))
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
                              (format str "(")
                              (format str "~A\\n" (car pddl-state))
                              (dolist (e (butlast (cdr pddl-state)))
                                (format str " ~A\\n" e))
                              (format str "~A)" (car (last pddl-state)))
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
        (dolist (l (cdr (assoc :step-state res)))
          ;; add node
          (setq nd (send gr :add-pddl-state-node l :node-name node-name))
          ;; add arc
          (when (and nd ns)
            (send gr :add-arc-from-to ns nd (format nil "~A" (pop plan))))
          (setq ns nd nd nil)
          )))
    gr))

(defun append-failed-result (result act &key (additional-conditions) ((:graph gr)) (node-name))
  (let ((ret (make-failed-condition result act))
        res lst
        (problem (copy-object *problem*)))

    ;;(pprint (list 'fail ret))

    (dolist (r ret)
      (send problem :initial-condition
            (append additional-conditions (cdr (assoc :initial-condition r))))
      (setq res (solve-pddl-planning *domain* problem :optimize-state nil))

      ;;(pprint (list 'result res))

      (push res lst)

      (setq gr (make-graph-from-pddl-results (list res) :graph gr :node-name node-name))

      (send gr :add-arc-from-to
            (send gr :search-node-from-pddl-state (cdr (assoc :src-condition r)))
            (send gr :search-node-from-pddl-state (cadr (assoc :step-state res)))
            (format nil "~A" (cdr (assoc :act-fail r)))))
    lst))

(defun add-failed-nodes (result failed-action-list &key ((:graph gr) (instance pddl-graph :init)))
  ;; for constant conditions
  (let ((def-init (send *problem* :initial-condition))
        (sol-init (result-condition->problem-condition (cdr (assoc :initial result))))
        additional-conditions)
    (setq additional-conditions (set-difference def-init sol-init :test #'equal))
    (when additional-conditions
      (warn ";; additonal conditions found ~A~%" additional-conditions))

    (let ((graph (make-graph-from-pddl-results (list result) :node-name :pprint :graph gr)) ;; original plan
          (results (list result)))
      (dolist (failed-action failed-action-list)
        (let ((count 0))
          (while (/= count (length (send graph :nodes)))
            (let (lst)
              (setq count (length (send graph :nodes)))
              (dolist (r results)
                (let ((ret (append-failed-result r failed-action :graph graph :node-name :pprint
                                                 :additional-conditions additional-conditions)))
                  (if ret (setq lst (append lst ret)))))
              (setq results (append lst results))))))
      ;; results
      graph)))
;;(setq *graph* (add-failed-nodes *result* (list 'checkclear 'confirm)))
;;(send *graph* :write-to-pdf "pddl.pdf")
