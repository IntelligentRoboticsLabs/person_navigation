(:durative-action guide_navigate
    :parameters (?r - robot ?from ?to - room ?d - door ?wp1 ?wp2 - waypoint ?p - person)
    :duration ( = ?duration 10)
    :condition (and

        (at start(waypoint_at ?wp1 ?from))
        (at start(waypoint_at ?wp2 ?to))

        (at start(robot_at_room ?r ?from))
        (at start(person_at_room ?p ?from))


        (at start(free_connected ?from ?to))
        (at start(robot_at ?r ?wp1))
        (at start(person_at ?p ?wp1))
    )
    :effect (and
      (at start(not(robot_at ?r ?wp1)))
      (at start(not(robot_at_room ?r ?from)))
      (at end(robot_at_room ?r ?to))
      (at end(robot_at ?r ?wp2))
      (at end(person_at_room ?p ?to))
      (at end(person_at ?p ?wp2))
    )
)

(:durative-action guide_move
  :parameters (?r - robot ?from ?to - waypoint ?room - room ?p - person)
  :duration ( = ?duration 10)
  :condition (and
    (at start(waypoint_at ?from ?room))
    (at start(waypoint_at ?to ?room))
    (at start(robot_at_room ?r ?room))
    (at start(robot_at ?r ?from))

    (at start(person_at ?p ?from))
    (at start(person_at_room ?p ?room))

  )
  :effect (and
    (at start(not(robot_at ?r ?from)))
    (at end(robot_at ?r ?to))

    (at start(not(person_at ?p ?from)))
    (at end(person_at ?p ?to))
  )
)

(:durative-action guide_cross
    :parameters (?r - robot ?from ?to - room ?d - door ?wp1 ?wp2 - waypoint ?p - person)
    :duration ( = ?duration 10)
    :condition (and
        (over all(door_opened ?d))

        (at start(waypoint_at ?wp1 ?from))
        (at start(waypoint_at ?wp2 ?to))

        (at start(robot_at_room ?r ?from))
        (at start(person_at_room ?p ?from))

        (at start(door_connected ?d ?from ?to ?wp1 ?wp2))
        (at start(robot_at ?r ?wp1))
        (at start(person_at ?p ?wp1))
    )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at start(not(robot_at_room ?r ?from)))
        (at end(robot_at_room ?r ?to))
        (at end(robot_at ?r ?wp2))

        (at start(not(person_at ?p ?wp1)))
        (at start(not(person_at_room ?p ?from)))
        (at end(person_at_room ?p ?to))
        (at end(person_at ?p ?wp2))
    )
)

(:durative-action guide_person
  :parameters (?r - robot ?wp1 - waypoint ?p - person)
  :duration ( = ?duration 500)
  :condition (and
    (at start(person_at ?p ?wp1))
 )
  :effect (and
    (at end(person_guided ?p ?wp1))
  )
)

(:durative-action follow_person
    :parameters (?r - robot ?p - person)
    :duration ( = ?duration 10)
    :condition (and
    )
    :effect (and
      (at end(person_followed ?r ?p))
    )
)

(:durative-action approach_person
    :parameters (?r - robot ?p - person ?wp - waypoint)
    :duration ( = ?duration 10)
    :condition (and
      (at start(robot_at ?r ?wp))
      (at start(person_at ?p ?wp))
    )
    :effect (and
      (at end(robot_near_person ?r ?p))
    )
)
