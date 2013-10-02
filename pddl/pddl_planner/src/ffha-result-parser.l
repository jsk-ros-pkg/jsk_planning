(defun parse-ff-result-file (fname)
  ;;(setq ret (parse-ff-result-file "result.txt"))
  (let ((lst (with-open-file
              (f fname)
              (parse-ff-result f))))
    (multiple-value-bind (init goal act) lst
      (list (cons :initial init)
            (cons :goal goal)
            (cons :action-list
                  (mapcar #'(lambda (x) (sub-parse x)) act))))))

(defun parse-ff-result-string (str)
  (let* ((strm (make-string-input-stream str))
         (lst (parse-ff-result strm)))
    (multiple-value-bind (init goal act) lst
      (list (cons :initial init)
            (cons :goal goal)
            (cons :action-list
                  (mapcar #'(lambda (x) (sub-parse x)) act))))))

(defun parse-ff-result (strm)
  (let (str ret initial-conds goal-conds)
    (while
        (or (string= "Action" (subseq str 0 6))
            (setq str (read-line strm nil nil)))
      ;;(pprint str)
      (when (string= "Action"
                     (subseq str 0 6))
        (let (act-lst preconds effects tmp exp)
          ;; action
          (let ((f (make-string-input-stream str))
                sym)
            (while (setq sym (read f nil nil)) (push sym act-lst))
            (setq act-lst (nreverse act-lst)))

          ;;
          (while (and (setq str (read-line strm nil nil))
                      (not (substringp "Action" str))
                      (not (substringp "-----" str)))
            (setq str (remove #\: str))
            (setq exp (read-from-string str nil nil))
            (cond
             ((eq exp 'Effects)
              (setq preconds (nreverse tmp)
                    tmp nil))
             ((eq exp 'Preconds)
              ;;
              )
             ((eq exp 'effect)
              (if tmp (push (nreverse tmp) effects))
              (setq tmp nil))
             ((eq exp 'increase) )
             ((null exp) )
             (t (push exp tmp)))
            )
          (if tmp (push (nreverse tmp) effects))
          (push (list act-lst preconds (nreverse effects)) ret))
        ) ;; /when action parse
      ;; parse initial
      (when (substringp "final initial state" str)
        (while (and (setq str (read-line strm nil nil))
                    (not (substringp "final goal" str)))
          ;;(setq str (remove #\: str))
          (unless (substringp "[RF0]" str)
            (let ((exp (read-from-string str nil nil)))
              (if exp (push exp initial-conds)))))
        (setq initial-conds (nreverse initial-conds)))
      ;; parse goal
      (when (substringp "final goal" str)
        (while (and (setq str (read-line strm nil nil))
                    (not (substringp "metric" str)))
          (let ((exp (read-from-string str nil nil)))
            (if exp (push exp goal-conds))))
        (setq goal-conds (nreverse goal-conds)))
      ) ;; /while
    (list initial-conds goal-conds (nreverse ret))))

;; remove garbage
(defun sub-parse (action)
  (multiple-value-bind (act pre eff) action
    (let (parsed-eff)
      (dolist (ef eff)
        (let (conds adds dels tmp)
          (dolist (e ef)
            (cond
             ((eq e 'conditions)
              )
             ((eq e 'adds)
              (if tmp (setq conds (nreverse tmp)))
              (setq tmp nil))
             ((eq e 'dels)
              (if tmp (setq adds (nreverse tmp)))
              (setq tmp nil))
             (t (push e tmp))))
          (if tmp (setq dels (nreverse tmp)))
          (if (or conds adds dels)
            (push (list conds adds dels) parsed-eff))))
      (list (cdr act) pre (nreverse parsed-eff)))))

;; plan
;; search action -> precond my be true
;; apply effects
;;    condition check -> add adds, del dels
(defun apply-act (act-now cond-now &optional (domain-representation *domain-rep*))
  (let ((applied-act
         (find-if #'(lambda (x) (equal (car x) act-now)) domain-representation)))
    ;;(pprint applied-act)
    (if applied-act
        (multiple-value-bind (act preconds effects) applied-act
          ;; check preconds??
          ;;(pprint (list preconds cond-now))

          (dolist (ef effects)
            (multiple-value-bind (conditions adds dels) ef
              ;;(pprint (list 'c conditions))
              (when (or (null conditions)
                        (= (length (intersection conditions cond-now :test #'equal))
                           (length conditions)))
                ;;(pprint (list 'e ef))
                (setq cond-now (union cond-now adds :test #'equal))
                (setq cond-now (set-difference cond-now dels :test #'equal)))
              ))
          cond-now))))

(defun apply-plan (initial plan action-list &key (debug))
  (let (cond-lst (cond-now initial) p)
    (setq p (pop plan))
    (if debug
        (push (cons p cond-now) cond-lst)
      (push cond-now cond-lst))
    (while p
      (setq cond-now (apply-act p cond-now action-list))
      (setq p (pop plan))
      (if debug
          (push (cons (if p p :goal) cond-now) cond-lst)
        (push cond-now cond-lst)))
    (nreverse cond-lst)))

;;(setq initial (cdr (assoc :initial ret)))
;;(setq goal (cdr (assoc :goal ret)))
;;(setq plan (cdr (assoc :plan ret)))
;;(setq action-list (cdr (assoc :action-list ret)))
;;(apply-plan (cdr (assoc :initial *result*)) (cdr (assoc :plan *result*)) (cdr (assoc :action-list *result*)) :debug t)
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

(defun result-condition->problem-condition (rcond)
  (let (ret)
    (setq ret (mapcar #'(lambda (x) (if (substringp "not-" (string (car x)))
                                        (let ((str (subseq (string (car x)) 4))
                                              ret)
                                          (list 'not (cons (intern (string-upcase str)) (cdr x))))
                                      x)) rcond))
    ;; bad knowhow ;; ffha should end segmentation fault if last of initial condition is (not (....))
    (setq ret (sort-condition ret))
    (if (equal (caar (last ret)) 'not)
        (if (equal (caar ret) 'not)
            (warn ";; all conditions are negative !!!~%")
          (setq ret (nreverse ret))))
    ret))
(defun problem-condition->result-condition (pcond)
  (let (ret)
    (setq ret (mapcar #'(lambda (x)
                          (if (eq (car x) 'not)
                              (let ((e (cadr x)))
                                (cons (read-from-string (format nil "not-~A" (car e))) (cdr e)))
                            x)) pcond))
    ret))

(defmacro add-conditions-to (elem additional-condition)
  `(sort-condition (remove-duplicates (append ,elem ,additional-condition) :test #'equal)))

(defun add-conditions-to-result (result additional-condition)
  (let ((stp (cdr (assoc :step-state result)))
        (ini (cdr (assoc :initial result)))
        ret)
    (if stp (setq result (delete (assoc :step-state result) result)))
    (setq result (delete (assoc :initial result) result))

    (setq ini (add-conditions-to ini additional-condition))
    (when stp
      (dolist (s stp)
        (push (add-conditions-to s additional-condition) ret))
      (setq ret (nreverse ret)))

    (setq result (append (list (cons :initial ini)) result))
    (when stp
      (push (cons :step-state ret) result))
    result))

(defun make-failed-condition (result act &key (act-fail (intern (string-upcase (format nil "~A_f" (string act))))))
  (let ((plan
         (apply-plan (cdr (assoc :initial result))
                     (cdr (assoc :plan result))
                     (cdr (assoc :action-list result)) :debug t))
        pcond ret)
    (while plan
      (setq pcond (find-if #'(lambda (x) (unless (atom (car x)) (equal (caar x) act))) plan))
      (setq plan (cdr (member pcond plan :test #'equal)))
      (when pcond
        (let* ((swap-act (cons act-fail (cdar pcond)))
               (icond (apply-act swap-act (cdr pcond) (cdr (assoc :action-list result)))))
          (if icond
              (push
               (list
                (cons :initial-condition (result-condition->problem-condition icond))
                (cons :act (car pcond))
                (cons :act-fail swap-act)
                (cons :src-condition (sort-condition (cdr pcond)))
                (cons :dst-condition (sort-condition icond)))
               ret)))))
    (nreverse ret)))

(provide :ffha-result-parser)
