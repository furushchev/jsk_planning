;; utility library for eus to use pddl-planner

;; automatically load roseus and ros message packages
(ros::roseus-add-msgs "roseus")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "pddl_msgs")

;; class
(defclass pddl-module
  :super propertied-object
  :slots ()
  )

;; '((a . <a>) (b . <b>) (aa . <a>))
(defclass pddl-problem
  :super pddl-module
  :slots (problem-name
          initial-condition
          goal-condition
          domain-name
          metric
          obj                           ;((name . type) (name . type) ...)
          ))

(defmethod pddl-problem
  (:init (&key (name nil) (domain nil) ((:metric m) nil) ((:objects o) nil))
    (setq problem-name name)
    (setq domain-name domain)
    (setq metric m)
    (setq obj o)
    self)
  ;; accessors
  (:name (&optional name)
    (if name (setq problem-name name))
    problem-name)
  (:domain (&optional name)
    (if name (setq domain-name name))
    domain-name)
  (:metric (&optional m)
    (if m (setq metric m))
    metric)
  (:objects (&optional o)
    (if o (setq obj o))
    obj)
  (:initial-condition (&optional (condition-list nil))
    (if condition-list (setq initial-condition condition-list))
    initial-condition)
  (:goal (&optional (condition-list nil))
    (if condition-list (setq goal-condition condition-list))
    goal-condition)
  (:goal-condition (&optional (condition-list nil)) ;alias to :goal
    (send self :goal condition-list))
  (:problem-sexp ()
    `(define (problem ,problem-name)
       (:domain ,domain-name)
       (:objects
         ,@(let ((types (remove-duplicates (mapcar #'cdr obj))))
             (mapcar #'(lambda (_type)
                         (append
                          (mapcar #'car
                                  (remove-if-not
                                   #'(lambda (x)
                                       (eq (cdr x) _type))
                                   obj))
                          (list '- _type)))
                     types)))
       (:init ,initial-condition)
       (:goal (and ,@goal-condition))
       ,(if metric `(:metric ,@metric))
       ))
  (:write-to-file (file)
    (let ((sexp (send self :problem-sexp)))
      (with-open-file (f file :direction :output)
        (pprint sexp f))))
  (:print ()
    (pprint (send self :problem-sexp)))
  (:add (&key ((:initial-condition init) nil)
              ((:goal-condition goal) nil)
              ((:metric m) nil)
              ((:objects o) nil))
    (if init (push init initial-condition))
    (if goal (push goal goal-condition))
    (if m (push m metric))
    (if o (push o obj))
    self)
  (:ros-message ()
    (let ((msg
           (instance pddl_msgs::PDDLProblem :init
              :name (string problem-name)
              :domain (string domain-name)
              :objects (mapcar #'(lambda (x)
                                   (instance pddl_msgs::PDDLObject
                                             :init
                                             :type (string (cdr x))
                                             :name (string (car x))))
                               obj)
              ;;we need to avoid `init' slot because of roseus naming rule
              :initial (mapcar #'(lambda (i) (format nil "~A" i))
                               initial-condition)
              :goal (format nil "~A" `(and ,@goal-condition))
              )))
      (if metric (send msg :metric (list->simple-string metric)))
      msg))
  )

(defclass pddl-domain
  :super pddl-module
  :slots (domain-name
          requirements
          types
          constants
          predicates
          functions
          action)
  )
(defmethod pddl-domain
  (:init (&key (name nil) ((:requirements req) nil))
    (if name (setq domain-name name))
    (if req (setq requirements req))
    self)
  (:name (&optional n)
    (if n (setq domain-name n))
    domain-name)
  (:requirements (&optional req)
    (if req (setq requirements req))
    requirements)
  (:types (&optional typ)
    (if typ (setq types typ))
    types)
  (:constants (&optional con)
    (if con (setq constants con))
    constants)
  (:predicates (&optional pre) ;;;(at-robby ?r - room) -> (at-robby (?r room))
                               ;;;(at ?b - ball ?r - room) -> (at (?b ball) (?r room)) ;; this is not implemented
    (if pre (setq predicates pre))
    predicates)
  (:functions (&optional fnc)
    (if fnc (setq functions fnc))
    functions)
  (:action (&optional ac)
    (if ac (setq action ac))
    action)
  (:use-durative-action ()
    (some #'(lambda (x) (derivedp x pddl-durative-action)) action))
  (:add (&key
         ((:requirements req) nil)
         ((:types typ) nil)
         ((:constants con) nil)
         ((:predicates pre) nil)
         ((:functions fnc) nil)
         ((:action ac) nil))
    (if req (setq requirements (append requirements (list req))))
    (if typ (setq types        (append types        (list typ))))
    (if con (setq constants    (append constants    (list con))))
    (if pre (setq predicates   (append predicates   (list pre))))
    (if fnc (setq functions    (append functions    (list fnc))))
    (if ac  (setq action       (append action       (list ac))))
    self)
  (:domain-sexp ()
    `(define (domain ,domain-name)
       (:requirements ,@requirements)
       (:types ,@types)
       (:constants ,@constants)
       (:predicates ,@predicates)
       (:functions ,@functions)
       ,@(send-all action :action-sexp)
       ))
  (:print ()
    (pprint (send self :domain-sexp)))
  (:write-to-file (fname)
    (with-open-file (f fname :direction :output)
      (pprint (send self :domain-sexp) f)))
  (:ros-message ()
    (instance pddl_msgs::PDDLDomain :init
              :name (string domain-name)
              :requirements (list->simple-string requirements)
              :types (mapcar #'string types)
              :constants (mapcar #'string constants)
              :predicates
              (mapcar
               #'(lambda (x) (format nil "~A" x))
               predicates)
              :functions
              (mapcar
               #'(lambda (x) (format nil "~A" x))
               functions)
              :actions (send-all action :ros-message)))
  ) ;; pddl-domain

(defclass pddl-action
  :super pddl-module
  :slots (name
          parameters
          precondition
          effect))
(defmethod pddl-action
  (:init (&key ((:name n) nil)
               ((:parameters par) nil)
               ((:precondition pre) nil)
               ((:effect ef) nil)
               )
    (if n (setq name n))
    (if pre (setq precondition pre))
    (if par (setq parameters par))
    (if ef (setq effect ef))
    self)
  (:name (&optional n)
    (if n (setq name n))
    name)
  (:parameters (&optional par)  ;;; (?obj0 - type0 ?obj1 - type1) -> ((?obj0 type0) (?obj1 type1))
                                ;;; (?obj0 ?obj1 - type0 ?obj2 - type1) -> ((?obj ?obj type) (?obj2 type1))
                                ;;; (?obj0 ?obj1) -> ((?obj0) (?obj1))
    (if par (setq parameters par))
    parameters)
  (:precondition (&optional pre)
    (if pre (setq precondition pre))
    precondition)
  (:effect (&optional ef)
    (if ef (setq effect ef))
    effect)
  (:add (&key ((:effect ef) nil)
              ((:parameters par) nil)
              ((:precondition pre) nil)
              )
    (if ef  (setq effect       (append effect       (list ef))))
    (if par (setq parameters   (append parameters   (list par))))
    (if pre (setq precondition (append precondition (list par))))
    self)
  (:action-sexp ()
    `(:action ,name
       :parameters ,(flatten (mapcar #'(lambda (x)
                                         (if (= (length x) 1)
                                             (list (car x))
                                           (append (butlast x) '(-) (last x))))
                                     parameters))
       :precondition (and ,@precondition)
       :effect (and ,@effect)
       ))
  (:ros-message ()
    (instance pddl_msgs::PDDLAction :init
              :name (string name)
              :parameters
              (if parameters
                  (list->simple-string
                   (list (flatten (mapcar #'(lambda (x)
                                              (if (= (length x) 1)
                                                  (list (car x))
                                                  (append (butlast x) '(-) (last x))))
                                          parameters))))
                   "()")
              :precondition
              (format nil "~A" `(and ,@precondition))
              :effect
              (format nil "~A" `(and ,@effect))
              ))
  ) ;; pddl-action

(defclass pddl-durative-action
    :super pddl-action
  :slots (duration))
(defmethod pddl-durative-action
  (:init (&rest args
          &key ((:name n) nil)
            ((:parameters par) nil)
            ((:precondition pre) nil)
            ((:effect ef) nil)
            ((:duration dur) nil))
   (setq duration dur)
   (send-super :init :name n :parameters par :precondition pre :effect ef))
  (:duration (&optional dur)
   (if dur (setq duration dur)) dur)
  (:add (&rest args ((:duration dur) nil) &allow-other-keys)
   (send self :duration dur)
   (send-super* :add args))
  (:action-exp ()
   `(:durative-action ,name
     :parameters ,(flatten (mapcar #'(lambda (x)
                                       (if (= (length x) 1)
                                           (list (car x))
                                           (append (butlast x) '(-) (last x))))
                                   parameters))
     :duration (= ?duration ,duration)
     :condition (and ,@precondition)
     :effect (and ,@effect)))
  (:ros-message ()
    (instance pddl_msgs::PDDLAction :init
              :name (string name)
              :parameters
              (if parameters
                  (list->simple-string
                   (list (flatten (mapcar #'(lambda (x)
                                              (if (= (length x) 1)
                                                  (list (car x))
                                                  (append (butlast x) '(-) (last x))))
                                          parameters))))
                  "()")
              :action_duration
              (format nil "(= ?duration ~A)" duration)
              :precondition
              (format nil "~A" `(and ,@precondition))
              :effect
              (format nil "~A" `(and ,@effect))
              ))
  (:start-condition ()
    (remove-if #'null
      (mapcar #'(lambda (c)
                  (cond
                    ((equal (subseq c 0 2) '(at start))
                     (caddr c))
                    ((equal (subseq c 0 2) '(at end))
                     nil)
                    ((equal (subseq c 0 2) '(over all))
                     (caddr c))
                    (t c))) precondition)))
  (:start-effect ()
    (remove-if #'null
      (mapcar #'(lambda (c)
                  (cond
                    ((equal (subseq c 0 2) '(at start))
                     (caddr c))
                    ((equal (subseq c 0 2) '(at end))
                     nil)
                    ((equal (subseq c 0 2) '(over all))
                     (caddr c))
                    (t c))) effect)))
  (:end-effect ()
    (remove-if #'null
      (mapcar #'(lambda (c)
                  (cond
                    ((equal (subseq c 0 2) '(at start))
                     nil)
                    ((equal (subseq c 0 2) '(at end))
                     (caddr c))
                    ((equal (subseq c 0 2) '(over all))
                     (caddr c))
                    (t c))) effect)))
  ) ;; pddl-durative-action


;; macro for define domain and problem
(defmacro define-pddl-problem (sym
                               &key
                               name domain objects
                               initial-condition goal-condition)
  (let ((tmp (gensym)))
    `(setq ,sym (let ((,tmp (instance pddl-problem :init)))
                  (send ,tmp :name ',name)
                  (send ,tmp :domain ',domain)
                  (send ,tmp :objects ',objects)
                  (send ,tmp :initial-condition ',initial-condition)
                  (send ,tmp :goal ',goal-condition)
                  ,tmp))))

(defun make-pddl-action (action-spec)
  (destructuring-bind
      (name &key parameters precondition effect)
      action-spec
    (instance pddl-action :init
              :name (string name)
              :parameters parameters
              :precondition precondition
              :effect effect)))

(defmacro define-pddl-domain (sym
                              &key
                              name requirements
                              types predicates
                              actions)
  (let ((tmp (gensym)))
    `(setq ,sym (let ((,tmp (instance pddl-domain :init)))
                  (send ,tmp :name ',name)
                  (send ,tmp :requirements ',requirements)
                  (send ,tmp :types ',types)
                  (send ,tmp :predicates ',predicates)
                  ;; build actions
                  (send ,tmp :action
                        (mapcar #'make-pddl-action ',actions))
                  ,tmp))))

;; function for parse planning result


;; utility
(defun list->simple-string (x)
  "this function is not cool...."
  (let ((str (format nil "~A" x)))
    (subseq str 1 (1- (length str))))) ; remove ( and )

(defclass pddl-state
  :super pddl-module
  :slots (name
          domain
          states
          actions
          objects
          tl-actions
          relationship-lst
          all-relationship-lst
          )
  )

(defmethod pddl-state
  (:init (&key ((:name n) nil)
               ((:domain dm) nil)
               ((:initial-state st) nil)
               ((:actions acts) nil)
               ((:objects objs) nil))
   (setq name n
         domain dm
         objects objs
         actions acts)
            (ros::ros-info "actions: ~A" actions)
   (send self :initial-state st)
   self)
  (:name (&optional n) (if n (setq name n)) name)
  (:domain (&optional dm) (if dm (setq domain dm)) domain)
  (:actions (&optional ac) (if ac (setq actions ac)) actions)
  (:use-durative-action () (send domain :use-durative-action))
  (:initial-state (&optional st)
    (if st (setq states (list (remove-if #'(lambda (s) (equal (car s) 'not)) st))))
    (car states))
  (:objects (&optional objs)
    (if objs (setq objects obj))
    (let ((objs (copy-object objects)))
      (if domain
          ;; merge constants defined in domain
          (let* ((cnst (copy-object (send domain :constants))) pos type)
            (while (setq pos (position '- cnst))
                   (setq type (elt cnst (1+ pos)))
                   (dotimes (i pos)
                     (push (cons (pop cnst) type) objs))
                   (pop cnst)    ;; -
                   (pop cnst)))) ;; type
      (unique objs :test #'equal)))
  (:states (&rest args)
    (forward-message-to states args))
  (:step-actions ()
   "((1.0
      ((end (movetotable robot1)) (start (pick robot1 dish tablecloth))))
     (11.0
      ((end (pick robot1 dish tablecloth))
       (start (pick robot1 cup tablecloth))))
     (21.0
      ((end (pick robot1 cup tablecloth))
       (start (pick robot1 tablecloth table))))
     (31.0 ((end (pick robot1 tablecloth table)))))"
   (if (send self :use-durative-action)
       (let (steps)
         (dolist (act (send self :actions))
           (push `(,(car act) start ,(cddr act)) steps)
           (push `(,(+ (car act) (cadr act)) end ,(cddr act)) steps))
         (let ((temp-steps (copy-object (sort steps #'>= #'car)))
               tm ac ac-lst)
           (setq steps nil)
           (while (setq ac (pop temp-steps))
             (cond ((null ac-lst) (setq ac-lst (list (cdr ac)) tm (car ac)))
                   ((eq tm (car ac)) (push (cdr ac) ac-lst))
                   (t (push (list tm ac-lst) steps)
                      (setq ac-lst (list (cdr ac)) tm (car ac))))))
         steps)
       (send self :actions)))
  (:find-action (act)
   (find-if #'(lambda (ac)
                (eq (read-from-string (send ac :name))
                    (if (derivedp ac pddl-durative-action)
                        (caddr act)  ;; (start-time end-time name params)
                        (car act)))) ;; (name params)
            (send domain :action)))
  (:find-action (act)
   (let ((names (if (numberp (car act)) (mapcar #'cadr (cdr act)) (list (car act)))))
     (remove-if-not #'(lambda (ac) (memq (read-from-string (send ac :name)) names)) (send domain :action))))
  (:step-states ()
   (unless (and (send self :initial-state)
                (send self :actions))
     (return-from :step-states nil))
   (let ((st (list (send self :initial-state))))
     (dolist (act (send self :step-actions))
       (setq st (append st (send self :apply-action (car (last st)) act))))
     (setq states st)
     states))
  (:apply-action (st act)
    (let ((acts (send self :find-action act))
          (mid-state (copy-object st))
          params)
      (unless action
        (ros::ros-error ":apply-action : Action ~A not found in domain" act)
        (return-from :apply-action nil))
      (setq params
            (cons (flatten (mapcar #'(lambda(x) (butlast x 1)) (send action :parameters)))
                  (if (derivedp action pddl-durative-action)
                      (cdddr act) (cdr act))))
      (unless (send self :check-condition st act params)
        (ros::ros-error ":apply-action : Condition error ~A" act)
        (return-from :apply-action nil))
      (ros::ros-debug "action matched: ~A params: ~A" action params)
      (if (derivedp action pddl-durative-action)
          ;; start and end
          (let (started-state ended-state)
            (ros::ros-debug "apply effect: ~A" (send action :start-effect))
            (dolist (e (send action :start-effect))
              (setq mid-state (send self :change-state mid-state e params)))
            (setq started-state (copy-object mid-state))
            (ros::ros-debug "started state: ~A" started-state)
            (ros::ros-debug "apply effect: ~A" (send action :end-effect))
            (dolist (e (send action :end-effect))
              (setq mid-state (send self :change-state mid-state e params)))
            (setq ended-state (copy-object mid-state))
            (ros::ros-debug "ended state: ~A" ended-state)
            (list started-state ended-state))
          ;; normal action
          (progn
            (ros::ros-info "apply effect: ~A" (send action :effect))
            (dolist (e (send action :effect))
              (setq mid-state (send self :change-state mid-state e params)))
            (list mid-state)))))
  (:check-condition (st act params)
   (let* ((action (send self :find-action act))
          (precond (if (derivedp action pddl-durative-action) (send action :start-condition) (send action :precondition))))
     (every #'identity (mapcar #'(lambda (p) (send self :check-state st p params)) precond))))
  (:resolve-params (st params)
   (mapcar #'(lambda (x)
               (let ((pos (position x (car params)))) (if pos (elt (cdr params) pos) x))) st))
  (:check-state (st s param)
    (let ((ss (send self :resolve-params s param)))
      ;; (ros::ros-info "ss: ~A s: ~A param: ~A" ss s param)
      (case (car ss)
        ('not
         (if (send self :check-state st (cadr ss) param)
             nil
           t))
        ('and
         (let ((ret (mapcar #'(lambda(x)
                                (send self :check-state st x param))
                            (cdr ss))))
           (null (some #'null ret))))
        ('or
         (let ((ret (mapcar #'(lambda(x)
                                (send self :check-state st x param))
                            (cdr ss))))
           (null (every #'null ret))))
        ('exists
         (let* ((v (cadr ss))
                (ret (mapcar #'(lambda(obj)
                                 (if (equal (caddr v) (cdr obj))
                                     (multiple-value-bind (va ar) param
                                       (send self :check-state st (caddr ss) (list (append va (list (car v))) (append ar (list (car obj))))))
                                   nil)
                                 )
                             objects)))
           (null (every #'null ret))))
        ('forall
         (let* ((v (cadr ss))
                (ret (mapcar #'(lambda(obj)
                                 (if (equal (caddr v) (cdr obj))
                                     (multiple-value-bind (va ar) param
                                       (send self :check-state st (caddr ss) (list (append va (list (car v))) (append ar (list (car obj))))))
                                   t)
                                 )
                             objects)))
           (null (some #'null ret))))
        ('when
         (if (send self :check-state st (cadr ss) param)
             (send self :check-state st (caddr ss) param)
           t))
        ('=
         (equal (cadr ss) (caddr ss)))
        (t
         (if (member ss st :test #'equal)
             t
           nil))
        )
      ))
  (:change-state (st e param)
    (let ((ee (send self :resolve-params e param))
          (tmp-st st))
      (case (car ee)
        ('increase
         st)
        ('decrease
         st)
        ('not
         (let ((dif (send self :resolve-params (cadr ee) param)))
           (set-difference st (list dif) :test #'equal)))
        ('and
         (dolist (eff (cdr ee))
           (setq tmp-st (send self :change-state tmp-st eff param)))
         tmp-st)
        ('forall
         (let ((v (cadr ee)))
           (dolist (obj objects)
             (when (equal (caddr v) (cdr obj))
               (multiple-value-bind (va ar) param
                 (setq tmp-st (send self :change-state tmp-st (caddr ee) (list (append va (list (car v))) (append ar (list (car obj)))))))))
           tmp-st))
        ('when
         (case (caadr ee)
           ('=
            (if (send self :check-state st (cadr ee) param)
                (send self :change-state st (caddr ee) param)
              st))
           ('not
            (if (null (send self :check-state st (car (cdadr ee)) param))
                (send self :change-state st (caddr ee) param)
              st))
           (t
            (if (send self :check-state st (cadr ee) param)
                (send self :change-state st (caddr ee) param)
              st))))
        (t
         (union st (list ee) :test #'equal)
         )
        )
      ))
  )

(provide :eus-pddl)
