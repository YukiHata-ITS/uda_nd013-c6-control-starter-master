/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
    this->Kpi = Kpi;
    this->Kii = Kii;
    this->Kdi = Kdi;
    this->output_lim_maxi = output_lim_maxi;
    this->output_lim_mini = output_lim_mini;

    this->cte_intg = 0.0f;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
    this->cte_prev = this->cte;
    this->cte = cte;
    this->cte_intg += cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;
    double cte_diff = this->cte - this->cte_prev;
//    double control_tmp = -(this->Kpi * this->cte) - (this->Kdi * cte_diff)/this->delta_time - (this->Kii * this->cte_intg)*this->delta_time;
//    double control_tmp = -(this->Kpi * this->cte) - (this->Kdi * cte_diff) - (this->Kii * this->cte_intg);
/* P */
//    double control_tmp = -(this->Kpi * this->cte); //- (this->Kdi * cte_diff)/this->delta_time - (this->Kii * this->cte_intg)*this->delta_time;
/* PD */
    double control_tmp = -(this->Kpi * this->cte) - (this->Kdi * cte_diff)/this->delta_time; // - (this->Kii * this->cte_intg)*this->delta_time;
/* PID */
//    double control_tmp = -(this->Kpi * this->cte) - (this->Kdi * cte_diff)/this->delta_time - (this->Kii * this->cte_intg)*this->delta_time;

    if(control_tmp > output_lim_maxi)
    {
        control = output_lim_maxi;
    }
    else if(control_tmp < output_lim_mini)
    {
        control = output_lim_mini;
    }
    else
    {
        control = control_tmp;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
    this->delta_time = new_delta_time;
}