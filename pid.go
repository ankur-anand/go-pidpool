package pidpool

import (
	"errors"
	"math"
	"sync"
	"time"
)

// PID implements PID controller as mentioned http://en.wikipedia.org/wiki/PID_controller.
type PID struct {
	mu sync.Mutex

	kp float64
	ki float64
	kd float64

	outputMin   float64
	outputMax   float64
	integralMin float64
	integralMax float64

	setPoint   float64
	prevValue  float64
	integral   float64
	prevError  float64
	lastUpdate time.Time
	deadBand   float64
}

// NewPID returns a new PID controller with the given gains and dead-band.
func NewPID(kp, ki, kd, deadBand float64) *PID {
	return &PID{
		kp:          kp,
		ki:          ki,
		kd:          kd,
		deadBand:    deadBand,
		outputMin:   math.Inf(-1),
		outputMax:   math.Inf(1),
		integralMin: -100,
		integralMax: 100,
		lastUpdate:  time.Now(),
	}
}

// SetOutputLimits sets min and max output.
func (pid *PID) SetOutputLimits(min, max float64) error {
	if min > max {
		return errors.New("min output greater than max output")
	}
	pid.mu.Lock()
	defer pid.mu.Unlock()
	pid.outputMin, pid.outputMax = min, max

	return nil
}

// SetIntegralLimits clamps the running sum (anti-windup).
func (pid *PID) SetIntegralLimits(min, max float64) error {
	if min > max {
		return errors.New("min integral greater than max integral")
	}
	pid.mu.Lock()
	defer pid.mu.Unlock()
	pid.integralMin, pid.integralMax = min, max

	// clamp.
	if pid.integral > pid.integralMax {
		pid.integral = pid.integralMax
	} else if pid.integral < pid.integralMin {
		pid.integral = pid.integralMin
	}

	return nil
}

// SetSetPoint sets the PID setPoint.
func (pid *PID) SetSetPoint(val float64) {
	pid.mu.Lock()
	defer pid.mu.Unlock()
	pid.setPoint = val
}

// GetSetPoint returns the current setPoint.
func (pid *PID) GetSetPoint() float64 {
	pid.mu.Lock()
	defer pid.mu.Unlock()
	return pid.setPoint
}

// SetPID sets the PID gains.
func (pid *PID) SetPID(kp, ki, kd float64) {
	pid.mu.Lock()
	defer pid.mu.Unlock()
	pid.kp, pid.ki, pid.kd = kp, ki, kd
}

// GetPID returns the PID gains.
func (pid *PID) GetPID() (float64, float64, float64) {
	pid.mu.Lock()
	defer pid.mu.Unlock()
	return pid.kp, pid.ki, pid.kd
}

// Update runs the PID calculation. Uses wall time for dt.
// You can also call UpdateDuration if you want to supply dt explicitly.
func (pid *PID) Update(value float64) float64 {
	pid.mu.Lock()
	defer pid.mu.Unlock()

	now := time.Now()
	dt := now.Sub(pid.lastUpdate).Seconds()
	pid.lastUpdate = now

	return pid.updateInternal(value, dt)
}

// UpdateDuration allows custom duration between updates.
func (pid *PID) UpdateDuration(value float64, dt float64) float64 {
	pid.mu.Lock()
	defer pid.mu.Unlock()
	return pid.updateInternal(value, dt)
}

func (pid *PID) updateInternal(value float64, dt float64) float64 {

	// proportional gain.
	err := pid.setPoint - value
	if math.Abs(err) < pid.deadBand {
		err = 0
	}

	// integral is total accumulated error over time.
	pid.integral += err * dt
	if pid.integral > pid.integralMax {
		pid.integral = pid.integralMax
	} else if pid.integral < pid.integralMin {
		pid.integral = pid.integralMin
	}

	derivative := 0.0
	if dt > 0 {
		// derivative on Measurement
		derivative = -(value - pid.prevValue) / dt
	}
	pid.prevValue = value

	output := pid.kp*err + pid.ki*pid.integral + pid.kd*derivative

	if output > pid.outputMax {
		output = pid.outputMax
	} else if output < pid.outputMin {
		output = pid.outputMin
	}

	pid.prevError = err

	return output
}
