#ifndef UXVCOS_CONTROLLER_PID_H
#define UXVCOS_CONTROLLER_PID_H

#include <math.h>
#include <rtt/PropertyBag.hpp>
#include <rtt/Property.hpp>

#include <uxvcos/Child.h>

#include "helper.h"
#include "Limit.h"

namespace uxvcos {
namespace Controller {

class PID {
  public:
    class Parameter : public Child {
    public:
      Parameter(const std::string& name = "PID", const std::string& description = "")
        : Child(name, description)
        , enabled("enabled", "Controller active", true)
        , T_in("T_in", "Time constant of the prefilter")
        , KP ("KP",  "Constant for proportional action")
        , KI ("KI",  "Constant for integral action")
        , KD ("KD",  "Constant for differential action")
        , Kw ("Kw",  "Constant for proportional direct action")
        , Kdw("Kdw", "Constant for differential direct action")
        , limit_P ("limit_P",  "Limits of the proportional error")
        , limit_I ("limit_I",  "Limits of the integral error")
        , limit_D ("limit_D",  "Limits of the differential error")
        , limit_w ("limit_w",  "Limits of the direct action")
        , limit_dw("limit_dw", "Limits of the differential direct action")
        , limit_u ("limit_u",  "Limits of the controller output")
      {
        properties()->addProperty(enabled);

        properties()->addProperty(T_in);
        properties()->addProperty(KP);
        properties()->addProperty(KI);
        properties()->addProperty(KD);
        properties()->addProperty(Kw);
        properties()->addProperty(Kdw);

        properties()->addProperty(limit_P);
        properties()->addProperty(limit_I);
        properties()->addProperty(limit_D);
        properties()->addProperty(limit_w);
        properties()->addProperty(limit_dw);
        properties()->addProperty(limit_u);
      }

      RTT::Property<bool> enabled;

      RTT::Property<double> T_in;
      RTT::Property<double> KP;
      RTT::Property<double> KI;
      RTT::Property<double> KD;
      RTT::Property<double> Kw;
      RTT::Property<double> Kdw;

      RTT::Property<Limit<double> > limit_P;
      RTT::Property<Limit<double> > limit_I;
      RTT::Property<Limit<double> > limit_D;
      RTT::Property<Limit<double> > limit_w;
      RTT::Property<Limit<double> > limit_dw;
      RTT::Property<Limit<double> > limit_u;
    };

private:
  Parameter *myparam;
  Parameter &param;

public:
    PID(bool angular = false, const std::string& name = "PID", const std::string& description = "")
      : myparam(new Parameter(name, description)), param(*myparam)
      , angular(angular)
    {
      reset();
    }

    PID(Parameter &parameter, bool angular = false)
      : myparam(0), param(parameter)
      , angular(angular)
    {
      reset();
    }

    ~PID() {
      delete myparam;
    }

    Parameter& parameters() {
      return param;
    }
    
    void reset() {
      y = NaN();
      dy = NaN();
      w1 = w2 = dw2 = NaN();
      u = u_w = u_dw = 0;
      e_P = e_I = e_I_old = e_D = 0;
      inLimits = 0;
    }

    double reset(double w) {
      reset();
      return set(w);
    }
    
    double set(double w) {
      this->w1 = w;
      return w;
    }

    double jump(double w) {
      this->w2 = w - (this->w1 - this->w2);
      this->w1 = w;
      return w;
    }

    double update(const double y, double dt, double pdy = NaN())
    {
      if (!param.enabled.get()) {
        return 0;
      }

      // Vorfilter
      if (!angular)
        w2 = helper::PT1(w1, w2, param.T_in, dt, &dw2);
      else
        w2 = helper::PT1_modulo(w1, w2, param.T_in, dt, &dw2, -M_PI, M_PI);

      // Vorsteuerung
      u_w  = param.limit_w.rvalue()(w2);     // direkt
      u_dw = param.limit_dw.rvalue()(dw2);   // Ableitung

      // Ableitung der Regelgröße y berechnen
      if (!isnan(pdy)) {
        dy = pdy;
        this->y = y;
      } else if (dt > 0.0) {
        dy = helper::diff(y, this->y, dt);
      }

      // Auf NaN prüfen
      if (isnan(y)) {
        u = NaN();
        return u;
      }

      // Regelabweichung berechnen
      if (!angular)
        e_P = param.limit_P.rvalue()(w2 - y);
      else
        e_P = param.limit_P.rvalue()(helper::modulo(w2 - y, -M_PI, M_PI));
      e_D = param.limit_D.rvalue()(dw2 - dy);
      e_I = param.limit_I.rvalue()(e_I_old + dt * e_P);
      
      return output();
    }

    double output() {
      // Reglerausgang
      u = param.KP * e_P + param.KI * e_I + param.KD * e_D + param.Kw * u_w + param.Kdw * u_dw;
      u = param.limit_u.rvalue()(u);
      inLimits = param.limit_u.rvalue().inLimits();
      if (inLimits == 0) inLimits = param.limit_I.rvalue().inLimits();

      // Anti Wind-up
      if (inLimits != 0) e_I = e_I_old;

      e_I_old = e_I;
      return u;
    }

    bool isValid() {
      return !isnan(u);
    }

  private:
    static double NaN() {
      return std::numeric_limits<double>::quiet_NaN();
    }

  public:
    double w1;              // ungefilterter Sollwert
    double w2;              // PT1-gefilterter Sollwert
    double dw2;             // Ableitung des Sollwerts nach PT1
    
    double y;               // Istwert
    double dy;              // Ableitung des Istwerts
    
    double e_P;             // aktueller P-Anteil
    double e_I;             // aktueller I-Anteil
    double e_I_old;         // I-Anteil vom vorherigen Schritt
    double e_D;             // aktueller D-Anteil
    double u_w;             // aktueller Wert der Vorsteuerung Kw
    double u_dw;            // aktueller Wert der Vorsteuerung Kdw

    double u;               // aktueller Reglerausgang
    int    inLimits;        // innerhalb der Begrenzung (-1/0/1) ?

    bool angular;
};

} // namespace Controller
} // namespace uxvcos

#endif // UXVCOS_CONTROLLER_PID_H
