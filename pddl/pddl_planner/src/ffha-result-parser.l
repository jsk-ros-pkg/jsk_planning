(defun parse-ff-result-file (fname)
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
             ((null exp) )
             (t (push exp tmp)))
            )
          (if tmp (push (nreverse tmp) effects))
          (push (list act-lst preconds effects) ret))
        ) ;; /when action parse
      ;; parse initial
      (when (substringp "final initial state" str)
        (while (and (setq str (read-line strm nil nil))
                    (not (substringp "final goal" str)))
          ;;(setq str (remove #\: str))
          (let ((exp (read-from-string str nil nil)))
            (if exp (push exp initial-conds))))
        (setq initial-conds (nreverse initial-conds)))
      ;; parse goal
      (when (substringp "final goal" str)
        (while (and (setq str (read-line strm nil nil))
                    (not (substringp "metric:" str)))
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
          (push (list conds adds dels) parsed-eff)))
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
                (setq cond-now (append cond-now adds))
                (setq cond-now (set-difference cond-now dels :test #'equal)))
              ))
          cond-now))))

(defun apply-plan (initial plan action-list)
  (let ((cond-now initial))
    (pprint cond-now)
    (dolist (p plan)
      (setq cond-now (apply-act p cond-now action-list))
      (pprint cond-now))
    cond-now))

;;(setq ret (parse-ff-result-file "result.txt"))
;;(setq initial (cdr (assoc :initial ret)))
;;(setq goal (cdr (assoc :goal ret)))
;;(setq action-list (cdr (assoc :action-list ret)))
