package pidpool_test

import (
	"testing"

	"github.com/ankur-anand/go-pidpool"
)

func TestSetPIDAndGetPID(t *testing.T) {
	p := pidpool.NewPID(1, 2, 3, 0.0)
	kp, ki, kd := p.GetPID()
	if kp != 1 || ki != 2 || kd != 3 {
		t.Fatalf("GetPID mismatch: got (%v,%v,%v)", kp, ki, kd)
	}

	p.SetPID(0.5, 0.1, 0.01)
	kp, ki, kd = p.GetPID()
	if kp != 0.5 || ki != 0.1 || kd != 0.01 {
		t.Fatalf("SetPID mismatch: got (%v,%v,%v)", kp, ki, kd)
	}
}

func TestSetAndGetSetPoint(t *testing.T) {
	p := pidpool.NewPID(1, 0, 0, 0.0)
	p.SetSetPoint(42)
	if got := p.GetSetPoint(); got != 42 {
		t.Fatalf("GetSetPoint: expected 42, got %v", got)
	}
}

func TestNewPID_ProportionalControl_OutputLimit(t *testing.T) {
	p := pidpool.NewPID(1, 0, 0, 0)
	if err := p.SetOutputLimits(2, 10); err != nil {
		t.Fatalf("SetOutputLimits err: %v", err)
	}

	p.SetSetPoint(100)
	out := p.UpdateDuration(0, 0.1)
	if out > 10 || out < 2 {
		t.Fatalf("output not clamped to [2,10]: got %v", out)
	}

	if err := p.SetOutputLimits(5, 4); err == nil {
		t.Fatalf("expected error for min>max")
	}
}
