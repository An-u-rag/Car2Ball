using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TargetLogic : MonoBehaviour
{
    private float _wallCollisionTime;
    [SerializeField] GameObject agent;
    void OnTriggerEnter(Collider collider) 
    {
        if (collider.gameObject.name == "Goal")
        {
            // ball made it into the goal. Get agent script and call BallInGoal method
            agent.GetComponent<CarGoalAgent>().BallInGoal();
        }
    }
    /*
    void OnCollisionStay(Collision collision)
    {
        //if (collision.gameObject.CompareTag("wall"))
        //{
        //    _wallCollisionTime += 1.0f;
        //    if (_wallCollisionTime >= 500.0f)
        //    {
        //        _wallCollisionTime = 0.0f;
        //        agent.GetComponent<CarGoalAgent>().BallTouchedWall();
        //    }
        //}
    }

    void OnCollisionExit(Collision collision)
    {
        if (collision.gameObject.CompareTag("wall"))
        {
            _wallCollisionTime = 0.0f;
        }
    }
    */
}
