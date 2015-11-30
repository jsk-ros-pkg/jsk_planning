;; dummy-action.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::advertise "move_to" std_msgs::String 1)
(ros::advertise "look" std_msgs::String 1)
(ros::advertise "push_button" std_msgs::String 1)

(setq *look-success?* t)
(setq *push-success?* t)

(defun move-to (spot)
  (ros::publish "move_to"
                (instance std_msgs::String :init
                          :data (format nil "(move-to ~A)" spot)))
  t)

(defun look (obj)
  (ros::publish "look"
                (instance std_msgs::String :init
                          :data (format nil "(look ~A)" obj)))
  (setq *look-success?* (null *look-success?*)))

(defun push-button (obj)
  (ros::publish "push_button"
                (instance std_msgs::String :init
                          :data (format nil "(push-button ~A)" obj)))
  (setq *push-success?* (null *push-success?*)))

(provide :dummy-action) ;; end of dummy-action.l
