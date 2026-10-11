(define (domain accountable-drone)
  (:requirements :strips :typing)

  (:types drone object location gate viewpoint)

  (:predicates
    (at ?d - drone ?l - location)
    (object-at ?o - object ?l - location)
    (found ?o - object)
    (visible ?o - object)
    (localized ?o - object)
    (inspected ?o - object ?v - viewpoint)
    (holding ?d - drone ?o - object)
    (gripper-empty ?d - drone)
    (reachable ?l - location)
    (platform ?l - location)
    (gate-at ?g - gate ?l - location)
    (gate-clear ?g - gate)
    (traversed ?g - gate)
    (placed ?o - object ?l - location)
    (landed ?d - drone)
  )

  (:action search
    :parameters (?o - object)
    :precondition (and)
    :effect (and (found ?o) (visible ?o) (localized ?o))
  )

  (:action inspect
    :parameters (?o - object ?v - viewpoint)
    :precondition (and (found ?o) (visible ?o) (localized ?o))
    :effect (inspected ?o ?v)
  )

  (:action navigate
    :parameters (?d - drone ?from - location ?to - location)
    :precondition (and (at ?d ?from) (reachable ?to))
    :effect (and (not (at ?d ?from)) (at ?d ?to))
  )

  (:action pick
    :parameters (?d - drone ?o - object ?l - location)
    :precondition (and
      (at ?d ?l) (object-at ?o ?l) (visible ?o)
      (localized ?o) (gripper-empty ?d))
    :effect (and
      (holding ?d ?o) (not (gripper-empty ?d)) (not (object-at ?o ?l)))
  )

  (:action traverse-gate
    :parameters (?d - drone ?g - gate ?l - location)
    :precondition (and (at ?d ?l) (gate-at ?g ?l) (gate-clear ?g))
    :effect (traversed ?g)
  )

  (:action place
    :parameters (?d - drone ?o - object ?l - location)
    :precondition (and (at ?d ?l) (holding ?d ?o) (platform ?l))
    :effect (and
      (placed ?o ?l) (object-at ?o ?l) (gripper-empty ?d)
      (not (holding ?d ?o)))
  )

  (:action land
    :parameters (?d - drone)
    :precondition (and)
    :effect (landed ?d)
  )
)
