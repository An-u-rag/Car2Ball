using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TargetLogic : MonoBehaviour
{
    [SerializeField] GameObject agent;
    void OnTriggerEnter(Collider collider) 
    {
        if (collider.gameObject.name == "Goal")
        {
            // ball made it into the goal. Get agent script and call BallInGoal method
            agent.GetComponent<CarGoalAgent>().BallInGoal();
        }
    }

    void OnCollisionEnter(Collision collision) 
    {
        if (collision.gameObject.CompareTag("wall"))
        {
            agent.GetComponent<CarGoalAgent>().BallTouchedWall();
        }
    }
}
