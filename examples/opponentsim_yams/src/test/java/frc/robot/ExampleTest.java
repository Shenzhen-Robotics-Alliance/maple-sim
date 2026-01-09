package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

class ExampleTest
{
  // To learn more about how to write unit tests, see the
  // JUnit 5 User Guide at https://junit.org/junit5/docs/current/user-guide/

  @Test
  // While optional, Display Names allow for clearer test purpose & intent naming via more human-readable names
  @DisplayName("String.toLoweCase should handle mixed case and return all lower case")
  void toLowerCaseHandlesMixedCase()
  {
    assertEquals("robot", "Robot".toLowerCase());
  }

  @Test
  @DisplayName("2 + 2 should return 4")
  void twoPlusTwoShouldEqualFour()
  {
    assertEquals(4, 2 + 2);
  }
}
